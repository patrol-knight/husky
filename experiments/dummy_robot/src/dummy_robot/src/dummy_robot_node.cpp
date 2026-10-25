#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "dummy_robot/lanelet_router.hpp"

namespace dummy_robot
{

namespace
{

constexpr double kArrivalToleranceM = 0.3;

using SteadyClock = std::chrono::steady_clock;

}  // namespace

class DummyRobotNode : public rclcpp::Node
{
public:
  explicit DummyRobotNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("dummy_robot_node", options)
  {
    const double update_rate_hz = declare_parameter<double>("update_rate_hz", 10.0);
    x_ = declare_parameter<double>("x", -55.01039897722149);
    y_ = declare_parameter<double>("y", 65.38078806758735);
    yaw_ = declare_parameter<double>("yaw", 0.0);
    battery_cycle_sec_ = declare_parameter<double>("battery_cycle_sec", 600.0);
    speed_mps_ = declare_parameter<double>("speed_mps", 1.5);
    const auto map_osm_path = declare_parameter<std::string>("map_osm_path", "/maps/lanelet2_map.osm");
    const auto map_projector_path =
      declare_parameter<std::string>("map_projector_path", "/maps/map_projector_info.yaml");

    try {
      router_.emplace(map_osm_path, map_projector_path);
      RCLCPP_INFO(get_logger(), "Loaded %zu lanelets from %s", router_->lanelet_count(), map_osm_path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "LaneletRouter unavailable; goals will be rejected: %s", e.what());
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/dummy_robot/odom", 10);
    battery_pub_ = create_publisher<std_msgs::msg::Float32>("/dummy_robot/battery_percent", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/dummy_robot/operational_state", 10);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/dummy_robot/goal", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_goal(*msg);});
    cancel_sub_ = create_subscription<std_msgs::msg::Header>(
      "/dummy_robot/cancel", 10, [this](std_msgs::msg::Header::ConstSharedPtr msg) {on_cancel(*msg);});

    start_time_ = SteadyClock::now();
    last_tick_ = start_time_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz), [this]() {tick();});

    RCLCPP_INFO(get_logger(), "dummy_robot_node started at (%.2f, %.2f)", x_, y_);
  }

private:
  // goal and cancel share one monotonic sequence (header.stamp.sec) stamped by
  // zenoh_bridge, since DDS doesn't order delivery across two topics.
  bool accept_seq(int32_t seq, const char * kind)
  {
    if (seq <= last_seq_) {
      RCLCPP_WARN(get_logger(), "Ignoring out-of-order/stale %s (seq=%d <= %d)", kind, seq, last_seq_);
      return false;
    }
    last_seq_ = seq;
    return true;
  }

  void on_goal(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (!accept_seq(msg.header.stamp.sec, "goal")) {
      return;
    }
    const double gx = msg.pose.position.x;
    const double gy = msg.pose.position.y;
    if (!router_) {
      RCLCPP_ERROR(get_logger(), "Rejecting goal: map/lanelet2 unavailable");
      return;
    }

    auto path = router_->plan_route(x_, y_, gx, gy);
    if (!path || path->empty()) {
      RCLCPP_WARN(get_logger(), "No route to goal (%.2f, %.2f); ignoring", gx, gy);
      return;
    }

    path_ = std::move(*path);
    path_index_ = 0;
    operational_state_ = "navigating";
    RCLCPP_INFO(get_logger(), "New goal (%.2f, %.2f): %zu-point route", gx, gy, path_.size());
  }

  void on_cancel(const std_msgs::msg::Header & msg)
  {
    if (!accept_seq(msg.stamp.sec, "cancel")) {
      return;
    }
    path_.clear();
    path_index_ = 0;
    operational_state_ = "idle";
    RCLCPP_INFO(get_logger(), "Navigation cancelled; halted at (%.2f, %.2f)", x_, y_);
  }

  void tick()
  {
    const auto now = SteadyClock::now();
    const double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    advance(dt);
    publish_odom();
    publish_battery(std::chrono::duration<double>(now - start_time_).count());
    publish_operational_state();
  }

  void advance(double dt)
  {
    if (operational_state_ != "navigating" || path_.empty()) {
      return;
    }

    double remaining = speed_mps_ * dt;
    while (remaining > 0.0 && path_index_ < path_.size()) {
      const auto [tx, ty] = path_[path_index_];
      const double dist = std::hypot(tx - x_, ty - y_);
      if (dist <= kArrivalToleranceM) {
        ++path_index_;
        continue;
      }
      const double step = std::min(remaining, dist);
      yaw_ = std::atan2(ty - y_, tx - x_);
      x_ += (tx - x_) / dist * step;
      y_ += (ty - y_) / dist * step;
      remaining -= step;
      if (step < dist) {
        break;
      }
    }

    if (path_index_ >= path_.size()) {
      operational_state_ = "arrived";
      path_.clear();
    }
  }

  void publish_odom()
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    const double speed = operational_state_ == "navigating" ? speed_mps_ : 0.0;
    msg.twist.twist.linear.x = speed * std::cos(yaw_);
    msg.twist.twist.linear.y = speed * std::sin(yaw_);
    odom_pub_->publish(msg);
  }

  void publish_battery(double elapsed_sec)
  {
    const double cycle_pos = std::fmod(elapsed_sec, battery_cycle_sec_) / battery_cycle_sec_;
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(100.0 - cycle_pos * 80.0);  // 100 -> 20, repeating
    battery_pub_->publish(msg);
  }

  void publish_operational_state()
  {
    std_msgs::msg::String msg;
    msg.data = operational_state_;
    state_pub_->publish(msg);
  }

  double x_;
  double y_;
  double yaw_;
  double battery_cycle_sec_;
  double speed_mps_;

  std::optional<LaneletRouter> router_;
  std::string operational_state_ = "idle";
  std::vector<Point2d> path_;
  std::size_t path_index_ = 0;
  int32_t last_seq_ = 0;

  SteadyClock::time_point start_time_;
  SteadyClock::time_point last_tick_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr cancel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dummy_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dummy_robot::DummyRobotNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
