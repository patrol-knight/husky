#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "dummy_robot/lanelet_router.hpp"

// Stand-in for the Husky's Autoware (localization, planning adaptor, vehicle
// interface) and Clearpath driver: the same topics and types, so the real
// state_machine, aggregator_node and zenoh_bridge_node run on top of it.
namespace dummy_robot
{

namespace
{

using autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_planning_msgs::msg::RouteState;

constexpr double kArrivalToleranceM = 0.3;
constexpr double kBatteryPeriodSec = 1.0;
constexpr double kRetryPeriodSec = 1.0;
constexpr double kManualCommandTimeoutSec = 0.25;
constexpr double kManualHoldSec = 1.0;

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

rclcpp::QoS latched()
{
  return rclcpp::QoS(1).reliable().transient_local();
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

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
    wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/husky/platform/odom", 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/husky/platform/bms/state", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/husky/platform/emergency_stop", 10);
    lidar_pub_ = create_publisher<std_msgs::msg::Bool>("/sensing/lidar/obstacle_stop", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/imu_data", 10);
    localization_pub_ = create_publisher<LocalizationInitializationState>(
      "/localization/initialization_state", latched());
    route_pub_ = create_publisher<RouteState>("/planning/route_state", latched());
    request_pub_ = create_publisher<std_msgs::msg::String>("/planning/route_request_status", 10);
    operation_pub_ = create_publisher<OperationModeState>("/system/operation_mode/state", latched());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_goal(*msg);});
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {on_initial_pose(*msg);});
    cancel_sub_ = create_subscription<std_msgs::msg::Header>(
      "/husky/cancel", 10, [this](std_msgs::msg::Header::ConstSharedPtr) {on_cancel();});
    for (const char * topic :
      {"/husky/joy_teleop/cmd_vel", "/husky/rc_teleop/cmd_vel", "/husky/twist_marker_server/cmd_vel"})
    {
      manual_subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(
        topic, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_manual(*msg);}));
    }
    hook_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/route_reject", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {route_reject_ = msg->data;}));
    hook_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/localization_lost", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {on_localization_lost(msg->data);}));
    hook_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/estop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {estop_ = msg->data;}));
    hook_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/obstacle", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {obstacle_ = msg->data;}));
    hook_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/dummy_robot/control_fault", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {control_fault_ = msg->data;}));

    start_time_ = SteadyClock::now();
    last_tick_ = start_time_;
    last_battery_publish_ = start_time_;
    publish_localization(false);
    set_route(RouteState::UNSET);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / update_rate_hz), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(), "dummy_robot_node started at (%.2f, %.2f); localizing for %.1f s", x_, y_, init_delay_sec_);
  }

private:
  void on_goal(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (!localized()) {
      RCLCPP_WARN(get_logger(), "Ignoring goal: not localized");
      return;
    }
    if (manual_active(SteadyClock::now())) {
      RCLCPP_WARN(get_logger(), "Ignoring goal during a manual takeover");
      return;
    }
    if (pending_goal_) {
      RCLCPP_WARN(get_logger(), "Route request is still in progress; dropping the goal");
      return;
    }
    failed_goal_.reset();
    pending_goal_ = Point2d{msg.pose.position.x, msg.pose.position.y};
    pending_until_ = SteadyClock::now() + to_duration(route_request_sec_);
    publish_request("pending");
    set_route(route_ == RouteState::SET ? RouteState::REROUTING : RouteState::ROUTING);
  }

  std::optional<std::vector<Point2d>> plan(const Point2d & goal) const
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
    auto path = plan(goal);
    if (!path) {
      RCLCPP_WARN(get_logger(), "Route rejected for goal (%.2f, %.2f)", goal.first, goal.second);
      clear_route();
      failed_goal_ = goal;
      last_retry_ = SteadyClock::now();
      publish_request("rejected");
      return;
    }
    path_ = std::move(*path);
    path_index_ = 0;
    set_route(RouteState::SET);
    publish_request("accepted");
    RCLCPP_INFO(get_logger(), "Goal (%.2f, %.2f): %zu-point route", goal.first, goal.second, path_.size());
  }

  // Like the adaptor: a retry that plans clears the recovered route and reports accepted.
  void retry_failed_goal(SteadyClock::time_point now)
  {
    if (!failed_goal_ || pending_goal_ || seconds_between(last_retry_, now) < kRetryPeriodSec) {
      return;
    }
    last_retry_ = now;
    if (!plan(*failed_goal_)) {
      return;
    }
    failed_goal_.reset();
    publish_request("accepted");
    RCLCPP_INFO(get_logger(), "Retry planned; recovered route cleared");
  }

  void on_cancel()
  {
    pending_goal_.reset();
    failed_goal_.reset();
    publish_request("cancelled");
    clear_route();
    RCLCPP_INFO(get_logger(), "Cancel: route cleared at (%.2f, %.2f)", x_, y_);
  }

  // Like the vehicle interface: a teleop command takes over and clears the route.
  void on_manual(const geometry_msgs::msg::Twist & cmd)
  {
    last_manual_ = SteadyClock::now();
    manual_cmd_ = cmd;
    if (!failed_goal_ && (pending_goal_ || route_ != RouteState::UNSET)) {
      pending_goal_.reset();
      clear_route();
      RCLCPP_INFO(get_logger(), "Manual takeover: route cleared at (%.2f, %.2f)", x_, y_);
    }
  }

  bool manual_active(SteadyClock::time_point now) const
  {
    return last_manual_ && seconds_between(*last_manual_, now) < kManualHoldSec;
  }

  // Like Autoware: localization stops while it aligns, then comes back at the pose.
  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    const auto & pose = msg.pose.pose;
    x_ = pose.position.x;
    y_ = pose.position.y;
    yaw_ = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
    pending_goal_.reset();
    clear_route();
    localization_lost_ = false;
    aligned_at_ = SteadyClock::now() + to_duration(init_delay_sec_);
    RCLCPP_INFO(get_logger(), "Initial pose (%.2f, %.2f); localizing for %.1f s", x_, y_, init_delay_sec_);
  }

  void on_localization_lost(bool lost)
  {
    if (lost == localization_lost_) {
      return;
    }
    localization_lost_ = lost;
    publish_localization(localized());
    if (lost) {
      pending_goal_.reset();
      clear_route();
      RCLCPP_WARN(get_logger(), "Localization lost at (%.2f, %.2f)", x_, y_);
    }
  }

  void tick()
  {
    const auto now = SteadyClock::now();
    const double dt = seconds_between(last_tick_, now);
    last_tick_ = now;

    if (localized() != published_localized_) {
      publish_localization(localized());
    }
    if (pending_goal_ && now >= pending_until_) {
      finish_route_request();
    }
    retry_failed_goal(now);

    const double yaw_before = yaw_;
    linear_mps_ = 0.0;
    if (!estop_ && last_manual_ && seconds_between(*last_manual_, now) < kManualCommandTimeoutSec) {
      drive_manual(dt);
    } else if (autonomous(now) && !obstacle_) {
      advance(dt);
    }
    const double yaw_rate = dt > 0.0 ? wrap_angle(yaw_ - yaw_before) / dt : 0.0;
    publish_wheel_odom(yaw_rate);
    if (localized()) {
      publish_odom(yaw_rate);
      publish_sensors();
    }
    publish_operation_mode(now);
    std_msgs::msg::Bool estop;
    estop.data = estop_;
    estop_pub_->publish(estop);

    if (seconds_between(last_battery_publish_, now) >= kBatteryPeriodSec) {
      last_battery_publish_ = now;
      publish_battery(seconds_between(start_time_, now));
    }
  }

  bool autonomous(SteadyClock::time_point now) const
  {
    return !estop_ && !control_fault_ && !manual_active(now) &&
           (route_ == RouteState::SET || route_ == RouteState::REROUTING);
  }

  void drive_manual(double dt)
  {
    linear_mps_ = manual_cmd_.linear.x;
    yaw_ = wrap_angle(yaw_ + manual_cmd_.angular.z * dt);
    x_ += linear_mps_ * std::cos(yaw_) * dt;
    y_ += linear_mps_ * std::sin(yaw_) * dt;
  }

  void advance(double dt)
  {
    if (path_.empty()) {
      return;
    }
    linear_mps_ = speed_mps_;
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
      path_.clear();
      path_index_ = 0;
      linear_mps_ = 0.0;
      set_route(RouteState::ARRIVED);
      RCLCPP_INFO(get_logger(), "Arrived at (%.2f, %.2f)", x_, y_);
    }
  }

  bool localized() const
  {
    const auto now = SteadyClock::now();
    return !localization_lost_ && seconds_between(start_time_, now) >= init_delay_sec_ && now >= aligned_at_;
  }

  // Like the vehicle interface: a tracking fault lasts until the route changes.
  void clear_route()
  {
    control_fault_ = false;
    path_.clear();
    path_index_ = 0;
    set_route(RouteState::UNSET);
  }

  void set_route(uint8_t state)
  {
    if (state == route_) {
      return;
    }
    route_ = state;
    RouteState msg;
    msg.stamp = now();
    msg.state = state;
    route_pub_->publish(msg);
  }

  void publish_request(const char * status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    request_pub_->publish(msg);
  }

  void publish_localization(bool initialized)
  {
    published_localized_ = initialized;
    LocalizationInitializationState msg;
    msg.stamp = now();
    msg.state = initialized ? LocalizationInitializationState::INITIALIZED :
      LocalizationInitializationState::UNINITIALIZED;
    localization_pub_->publish(msg);
  }

  void publish_operation_mode(SteadyClock::time_point now_steady)
  {
    OperationModeState msg;
    msg.stamp = now();
    const bool auto_mode = autonomous(now_steady);
    msg.mode = estop_ ? OperationModeState::STOP :
      auto_mode ? OperationModeState::AUTONOMOUS : OperationModeState::LOCAL;
    msg.is_autoware_control_enabled = auto_mode;
    msg.is_stop_mode_available = true;
    msg.is_autonomous_mode_available = !estop_;
    msg.is_local_mode_available = true;
    operation_pub_->publish(msg);
  }

  void publish_sensors()
  {
    std_msgs::msg::Bool obstacle;
    obstacle.data = obstacle_;
    lidar_pub_->publish(obstacle);
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = "imu_link";
    imu.orientation.w = 1.0;
    imu_pub_->publish(imu);
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
    msg.twist.twist.linear.x = linear_mps_;
    msg.twist.twist.angular.z = yaw_rate;
    odom_pub_->publish(msg);
  }

  void publish_wheel_odom(double yaw_rate)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.twist.twist.linear.x = linear_mps_;
    msg.twist.twist.angular.z = yaw_rate;
    wheel_odom_pub_->publish(msg);
  }

  void publish_battery(double elapsed_sec)
  {
    const double cycle_pos = std::fmod(elapsed_sec, battery_cycle_sec_) / battery_cycle_sec_;
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.percentage = static_cast<float>(1.0 - cycle_pos * 0.8);
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
  SteadyClock::time_point aligned_at_{};
  double battery_cycle_sec_;
  double speed_mps_;
  double goal_map_tolerance_m_;
  double route_request_sec_;
  bool route_reject_ = false;
  bool localization_lost_ = false;
  bool published_localized_ = false;
  bool estop_ = false;
  bool obstacle_ = false;
  bool control_fault_ = false;
  double linear_mps_ = 0.0;
  std::optional<SteadyClock::time_point> last_manual_;
  geometry_msgs::msg::Twist manual_cmd_;

  std::optional<LaneletRouter> router_;
  uint8_t route_ = RouteState::UNKNOWN;
  std::vector<Point2d> path_;
  std::size_t path_index_ = 0;
  std::optional<Point2d> pending_goal_;
  SteadyClock::time_point pending_until_;
  std::optional<Point2d> failed_goal_;
  SteadyClock::time_point last_retry_;

  SteadyClock::time_point start_time_;
  SteadyClock::time_point last_tick_;
  SteadyClock::time_point last_battery_publish_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<LocalizationInitializationState>::SharedPtr localization_pub_;
  rclcpp::Publisher<RouteState>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr cancel_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> manual_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> hook_subs_;
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
