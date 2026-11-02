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
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "dummy_robot/lanelet_router.hpp"

// Stand-in for the Husky: the same topics, types and /robot_status values as
// Autoware + robot_state + the Clearpath driver, so aggregator_node and
// zenoh_bridge_node run unchanged against it.
namespace dummy_robot
{

namespace
{

constexpr double kArrivalToleranceM = 0.3;
constexpr double kStatusPeriodSec = 0.2;  // robot_state: 5 Hz
constexpr double kReachHoldSec = 0.2;     // robot_state: "reach" once, "ready" 0.2 s later
constexpr double kBatteryPeriodSec = 1.0;
constexpr double kRetryPeriodSec = 1.0;

using SteadyClock = std::chrono::steady_clock;

double seconds_between(SteadyClock::time_point from, SteadyClock::time_point to)
{
  return std::chrono::duration<double>(to - from).count();
}

SteadyClock::duration to_duration(double sec)
{
  return std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(sec));
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

enum class Mode
{
  kNotReady,
  kReady,
  kNavigate,
  kFailure,
  kManual,
};

const char * status_name(Mode mode)
{
  switch (mode) {
    case Mode::kNotReady: return "notready";
    case Mode::kReady: return "ready";
    case Mode::kNavigate: return "navigate";
    case Mode::kFailure: return "failure";
    case Mode::kManual: return "manual";
  }
  return "notready";
}

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
    init_delay_sec_ = declare_parameter<double>("init_delay_sec", 2.0);
    battery_cycle_sec_ = declare_parameter<double>("battery_cycle_sec", 600.0);
    speed_mps_ = declare_parameter<double>("speed_mps", 1.5);
    // Autoware rejects a goal that isn't on a lanelet; this is the slack allowed.
    goal_map_tolerance_m_ = declare_parameter<double>("goal_map_tolerance_m", 1.0);
    route_request_sec_ = declare_parameter<double>("route_request_sec", 0.5);
    const auto map_osm_path = declare_parameter<std::string>("map_osm_path", "/maps/lanelet2_map.osm");
    const auto map_projector_path =
      declare_parameter<std::string>("map_projector_path", "/maps/map_projector_info.yaml");

    try {
      router_.emplace(map_osm_path, map_projector_path);
      RCLCPP_INFO(get_logger(), "Loaded %zu lanelets from %s", router_->lanelet_count(), map_osm_path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "LaneletRouter unavailable; every goal will fail: %s", e.what());
    }

    // Autoware localization: published only once initialized.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
    // Clearpath BMS.
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/husky/platform/bms/state", 10);
    // robot_state: reliable + transient-local.
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/robot_status", rclcpp::QoS(1).reliable().transient_local());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_goal(*msg);});
    cancel_sub_ = create_subscription<std_msgs::msg::Header>(
      "/husky/cancel", 10, [this](std_msgs::msg::Header::ConstSharedPtr) {on_cancel();});
    // Test hook in place of a joystick: true = takeover, false = released.
    manual_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/manual", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {on_manual(msg->data);});
    route_reject_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/route_reject", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {route_reject_ = msg->data;});
    localization_lost_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/localization_lost", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {on_localization_lost(msg->data);});

    start_time_ = SteadyClock::now();
    last_tick_ = start_time_;
    last_status_publish_ = start_time_;
    last_battery_publish_ = start_time_;
    publish_status(status_name(mode_));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(), "dummy_robot_node started at (%.2f, %.2f); localizing for %.1f s", x_, y_,
      init_delay_sec_);
  }

private:
  void on_goal(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (mode_ == Mode::kNotReady || mode_ == Mode::kManual) {
      RCLCPP_WARN(get_logger(), "Ignoring goal while %s", status_name(mode_));
      return;
    }
    if (pending_goal_) {
      RCLCPP_WARN(get_logger(), "Dropping goal: a route request is in flight");
      return;
    }
    pending_goal_ = Point2d{msg.pose.position.x, msg.pose.position.y};
    pending_until_ = SteadyClock::now() + to_duration(route_request_sec_);
  }

  std::optional<std::vector<Point2d>> request_route(const Point2d & goal) const
  {
    const auto [gx, gy] = goal;
    if (route_reject_ || !router_ || router_->distance_to_map(gx, gy) > goal_map_tolerance_m_) {
      return std::nullopt;
    }
    auto path = router_->plan_route(x_, y_, gx, gy);
    if (path && path->empty()) {
      return std::nullopt;
    }
    return path;
  }

  void finish_route_request()
  {
    const Point2d goal = *pending_goal_;
    pending_goal_.reset();
    const auto [gx, gy] = goal;

    auto path = request_route(goal);
    if (!path) {
      RCLCPP_WARN(get_logger(), "Route rejected for goal (%.2f, %.2f): failure", gx, gy);
      clear_route();
      reach_until_.reset();
      failed_goal_ = goal;
      last_retry_ = SteadyClock::now();
      set_mode(Mode::kFailure);
      return;
    }

    // Covers ready, arrived (ready after reach), rerouting while navigating, and failure.
    failed_goal_.reset();
    path_ = std::move(*path);
    path_index_ = 0;
    reach_until_.reset();
    set_mode(Mode::kNavigate);
    RCLCPP_INFO(get_logger(), "New goal (%.2f, %.2f): %zu-point route", gx, gy, path_.size());
  }

  void retry_failed_goal(SteadyClock::time_point now)
  {
    if (mode_ != Mode::kFailure || !failed_goal_ || pending_goal_ ||
      seconds_between(last_retry_, now) < kRetryPeriodSec)
    {
      return;
    }
    last_retry_ = now;
    if (!request_route(*failed_goal_)) {
      return;
    }
    // The adaptor clears the recovered route: back to ready without driving.
    failed_goal_.reset();
    set_mode(Mode::kReady);
    RCLCPP_INFO(get_logger(), "Retry planned: failure cleared, route dropped");
  }

  void on_cancel()
  {
    pending_goal_.reset();
    failed_goal_.reset();
    clear_route();
    if (mode_ == Mode::kNavigate || mode_ == Mode::kFailure) {
      set_mode(Mode::kReady);
    }
    RCLCPP_INFO(get_logger(), "Cancel: route cleared at (%.2f, %.2f)", x_, y_);
  }

  void on_manual(bool active)
  {
    if (active) {
      // robot_state cancels the route on a takeover, and the operator drives out of a failure.
      pending_goal_.reset();
      failed_goal_.reset();
      clear_route();
      reach_until_.reset();
      set_mode(Mode::kManual);
    } else if (mode_ == Mode::kManual) {
      set_mode(localized() ? Mode::kReady : Mode::kNotReady);
    }
  }

  void on_localization_lost(bool lost)
  {
    if (lost == localization_lost_) {
      return;
    }
    localization_lost_ = lost;
    if (!lost) {
      return;
    }
    pending_goal_.reset();
    clear_route();
    reach_until_.reset();
    if (mode_ == Mode::kReady || mode_ == Mode::kNavigate) {
      set_mode(Mode::kNotReady);
    }
    RCLCPP_WARN(get_logger(), "Localization lost at (%.2f, %.2f)", x_, y_);
  }

  void tick()
  {
    const auto now = SteadyClock::now();
    const double dt = seconds_between(last_tick_, now);
    last_tick_ = now;

    if (mode_ == Mode::kNotReady && localized()) {
      set_mode(Mode::kReady);
    }
    if (pending_goal_ && now >= pending_until_) {
      finish_route_request();
    }
    retry_failed_goal(now);

    const double yaw_before = yaw_;
    advance(dt);
    if (localized()) {
      publish_odom(dt > 0.0 ? wrap_angle(yaw_ - yaw_before) / dt : 0.0);
    }

    if (seconds_between(last_battery_publish_, now) >= kBatteryPeriodSec) {
      last_battery_publish_ = now;
      publish_battery(seconds_between(start_time_, now));
    }

    if (reach_until_) {
      if (now >= *reach_until_) {
        reach_until_.reset();
        publish_status(status_name(mode_));
      }
    } else if (seconds_between(last_status_publish_, now) >= kStatusPeriodSec) {
      publish_status(status_name(mode_));
    }
  }

  void advance(double dt)
  {
    if (mode_ != Mode::kNavigate || path_.empty()) {
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
      clear_route();
      mode_ = Mode::kReady;
      // "reach" once; tick() sends "ready" kReachHoldSec later.
      publish_status("reach");
      reach_until_ = SteadyClock::now() + to_duration(kReachHoldSec);
      RCLCPP_INFO(get_logger(), "Reached goal at (%.2f, %.2f)", x_, y_);
    }
  }

  bool localized() const
  {
    return !localization_lost_ && seconds_between(start_time_, SteadyClock::now()) >= init_delay_sec_;
  }

  void clear_route()
  {
    path_.clear();
    path_index_ = 0;
  }

  void set_mode(Mode mode)
  {
    if (mode == mode_) {
      return;
    }
    mode_ = mode;
    if (!reach_until_) {
      publish_status(status_name(mode_));  // on change, like robot_state
    }
  }

  void publish_status(const char * status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    last_status_publish_ = SteadyClock::now();
  }

  void publish_odom(double yaw_rate)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    // Body frame, as Autoware's kinematic_state.
    msg.twist.twist.linear.x = mode_ == Mode::kNavigate ? speed_mps_ : 0.0;
    msg.twist.twist.angular.z = yaw_rate;
    odom_pub_->publish(msg);
  }

  void publish_battery(double elapsed_sec)
  {
    const double cycle_pos = std::fmod(elapsed_sec, battery_cycle_sec_) / battery_cycle_sec_;
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.percentage = static_cast<float>(1.0 - cycle_pos * 0.8);  // 1.0 -> 0.2, repeating
    msg.voltage = static_cast<float>(24.0 + 4.0 * msg.percentage);
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    battery_pub_->publish(msg);
  }

  double x_;
  double y_;
  double yaw_;
  double init_delay_sec_;
  double battery_cycle_sec_;
  double speed_mps_;
  double goal_map_tolerance_m_;
  double route_request_sec_;
  bool route_reject_ = false;
  bool localization_lost_ = false;

  std::optional<LaneletRouter> router_;
  Mode mode_ = Mode::kNotReady;
  std::vector<Point2d> path_;
  std::size_t path_index_ = 0;
  std::optional<SteadyClock::time_point> reach_until_;
  std::optional<Point2d> pending_goal_;
  SteadyClock::time_point pending_until_;
  std::optional<Point2d> failed_goal_;
  SteadyClock::time_point last_retry_;

  SteadyClock::time_point start_time_;
  SteadyClock::time_point last_tick_;
  SteadyClock::time_point last_status_publish_;
  SteadyClock::time_point last_battery_publish_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr route_reject_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_lost_sub_;
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
