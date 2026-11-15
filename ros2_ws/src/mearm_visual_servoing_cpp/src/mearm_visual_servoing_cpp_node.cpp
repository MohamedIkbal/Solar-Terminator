// Image-based visual servoing for the MeArm (C++ port of
// mearm_visual_servoing_node.py).
//
// Subscribes to the vision_msgs/Detection2D stream published by
// mearm_object_detection (/object_detection/detections) and converts the
// tracked object's pixel position/size into joint targets, streamed out on
// the same interfaces the rest of the stack expects:
//
//   * /arm_joint_commands                  (std_msgs/Float64MultiArray, 4 values)
//   * /arm_controller/joint_trajectory     (trajectory_msgs/JointTrajectory)
//   * /gripper_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kUnknownLabel[] = "unknown";

double mapRange(double value, double in_min, double in_max, double out_min, double out_max)
{
  double t = (in_max == in_min) ? 0.0 : (value - in_min) / (in_max - in_min);
  t = std::max(0.0, std::min(1.0, t));
  return out_min + t * (out_max - out_min);
}
}  // namespace

// ── Per-joint smoother ─────────────────────────────────────────────────────
// Supports two modes:
//   EMA:       first-order exponential smoothing (existing behaviour)
//   S-curve:   cubic smootherstep (6t^5 − 15t^4 + 10t^3) with zero
//              velocity at both ends of each move — much smoother for servos.
class JointSmoother
{
public:
  enum class Mode { EMA, SCURVE };

  JointSmoother(double initial_pos, Mode mode, double alpha, double duration_s)
    : pos_(initial_pos), target_(initial_pos), mode_(mode),
      alpha_(alpha), duration_s_(duration_s),
      scurve_t_(1.0), scurve_moving_(false) {}

  /// Call whenever a new target is computed (e.g. from object detection).
  void set_target(double target)
  {
    target_ = target;
    if (mode_ == Mode::SCURVE) {
      start_pos_ = pos_;
      scurve_t_ = 0.0;
      scurve_moving_ = true;
    }
  }

  /// Advance by dt_s seconds and return the smoothed position.
  double update(double dt_s)
  {
    if (mode_ == Mode::EMA) {
      double step = alpha_ * (target_ - pos_);
      pos_ += step;
      return pos_;
    }

    // S-curve mode
    if (!scurve_moving_) {
      return pos_;
    }
    scurve_t_ += dt_s / duration_s_;
    if (scurve_t_ >= 1.0) {
      scurve_t_ = 1.0;
      scurve_moving_ = false;
      pos_ = target_;
    } else {
      double s = smootherstep(scurve_t_);
      pos_ = start_pos_ + (target_ - start_pos_) * s;
    }
    return pos_;
  }

  double position() const { return pos_; }
  double target() const { return target_; }
  bool moving() const { return scurve_moving_; }

private:
  static double smootherstep(double t)
  {
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    return 6.0 * t5 - 15.0 * t4 + 10.0 * t3;
  }

  double pos_;
  double target_;
  double start_pos_;
  Mode mode_;
  double alpha_;
  double duration_s_;
  double scurve_t_;
  bool scurve_moving_;
};

class MeArmVisualServoingNode : public rclcpp::Node
{
public:
  MeArmVisualServoingNode()
  : Node("mearm_visual_servoing_node"), have_seen_once_(false), target_class_("")
  {
    declare_parameter("image_width", 640);
    declare_parameter("image_height", 480);
    declare_parameter("min_object_area", 400.0);
    declare_parameter("max_object_area", 150000.0);
    declare_parameter("min_confidence", 0.5);
    declare_parameter("smoothing_alpha", 0.10);
    declare_parameter("use_scurve_smoothing", true);
    declare_parameter("scurve_duration_s", 0.4);
    declare_parameter("scurve_min_delta_rad", 0.01);
    declare_parameter("deadband_px", 14.0);
    declare_parameter("object_lost_timeout_ms", 800);
    declare_parameter("control_rate_hz", 20.0);
    declare_parameter("max_joint_speed_rad_s", 0.3);
    declare_parameter("joint0_min", -1.2);
    declare_parameter("joint0_max", 1.2);
    declare_parameter("joint1_min", -1.57);
    declare_parameter("joint1_max", 1.0);
    declare_parameter("joint2_min", 0.0);
    declare_parameter("joint2_max", 1.57);
    declare_parameter("joint3_min", 0.0);
    declare_parameter("joint3_max", 0.42);
    declare_parameter("joint3_tracking", 0.0);

    image_width_ = get_parameter("image_width").as_int();
    image_height_ = get_parameter("image_height").as_int();
    min_object_area_ = get_parameter("min_object_area").as_double();
    max_object_area_ = get_parameter("max_object_area").as_double();
    min_confidence_ = get_parameter("min_confidence").as_double();
    smoothing_alpha_ = get_parameter("smoothing_alpha").as_double();
    use_scurve_smoothing_ = get_parameter("use_scurve_smoothing").as_bool();
    scurve_duration_s_ = get_parameter("scurve_duration_s").as_double();
    scurve_min_delta_rad_ = get_parameter("scurve_min_delta_rad").as_double();
    deadband_px_ = get_parameter("deadband_px").as_double();
    object_lost_timeout_ms_ = get_parameter("object_lost_timeout_ms").as_int();
    double control_rate_hz = get_parameter("control_rate_hz").as_double();
    max_joint_speed_rad_s_ = get_parameter("max_joint_speed_rad_s").as_double();
    control_rate_hz_ = control_rate_hz;

    j0_min_ = get_parameter("joint0_min").as_double();
    j0_max_ = get_parameter("joint0_max").as_double();
    j1_min_ = get_parameter("joint1_min").as_double();
    j1_max_ = get_parameter("joint1_max").as_double();
    j2_min_ = get_parameter("joint2_min").as_double();
    j2_max_ = get_parameter("joint2_max").as_double();
    j3_min_ = get_parameter("joint3_min").as_double();
    j3_max_ = get_parameter("joint3_max").as_double();
    j3_tracking_ = get_parameter("joint3_tracking").as_double();

    home_j0_ = 0.0;
    home_j1_ = (j1_min_ + j1_max_) / 2.0;
    home_j2_ = (j2_min_ + j2_max_) / 2.0;
    home_j3_ = (j3_min_ + j3_max_) / 2.0;

    smoothed_ = {home_j0_, home_j1_, home_j2_, home_j3_};
    smoothers_.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
      smoothers_.emplace_back(smoothed_[i],
        use_scurve_smoothing_ ? JointSmoother::Mode::SCURVE
                              : JointSmoother::Mode::EMA,
        smoothing_alpha_, scurve_duration_s_);
    }
    last_seen_ = now();

    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_joint_commands", 10);
    arm_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    gripper_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/gripper_controller/joint_trajectory", 10);

    detection_sub_ = create_subscription<vision_msgs::msg::Detection2D>(
      "/object_detection/detections", 10,
      std::bind(&MeArmVisualServoingNode::detectionCallback, this, std::placeholders::_1));
    target_class_sub_ = create_subscription<std_msgs::msg::String>(
      "/object_detection/target_class", 10,
      std::bind(&MeArmVisualServoingNode::targetClassCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MeArmVisualServoingNode::controlTick, this));

    RCLCPP_INFO(
      get_logger(),
      "mearm_visual_servoing_node (C++) started: image %ldx%ld, control %.1f Hz, "
      "lost timeout %ld ms",
      image_width_, image_height_, control_rate_hz, object_lost_timeout_ms_);
  }

private:
  void targetClassCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    target_class_ = msg->data;
    RCLCPP_INFO(get_logger(), "Visual servoing target class set to: '%s'", target_class_.c_str());
  }

  void detectionCallback(const vision_msgs::msg::Detection2D::SharedPtr msg)
  {
    const std::string & label = msg->id;
    double score = msg->results.empty() ? 0.0 : msg->results[0].hypothesis.score;

    if (label == kUnknownLabel || label == "none" || label.empty()) {
      return;
    }
    if (score < min_confidence_) {
      return;
    }
    if (!target_class_.empty() && label != target_class_) {
      return;
    }

    double nx = msg->bbox.center.position.x / static_cast<double>(image_width_);
    double ny = msg->bbox.center.position.y / static_cast<double>(image_height_);
    double area = msg->bbox.size_x * msg->bbox.size_y;

    // Deadband: ignore sub-pixel jitter so the arm doesn't hunt in place on a
    // perfectly still target.
    if (have_target_) {
      double dx_px = std::abs(nx * image_width_ - last_target_[0] * image_width_);
      double dy_px = std::abs(ny * image_height_ - last_target_[1] * image_height_);
      if (dx_px < deadband_px_ && dy_px < deadband_px_) {
        last_seen_ = now();
        have_seen_once_ = true;
        return;
      }
    }

    last_target_ = {nx, ny, area};
    have_target_ = true;
    last_seen_ = now();
    have_seen_once_ = true;
  }

  void controlTick()
  {
    double elapsed_ms = (now() - last_seen_).nanoseconds() / 1e6;

    double target_j0, target_j1, target_j2, target_j3;
    if (have_seen_once_ && elapsed_ms <= object_lost_timeout_ms_ && have_target_) {
      double nx = last_target_[0];
      double ny = last_target_[1];
      double area = last_target_[2];
      target_j0 = mapRange(nx, 0.0, 1.0, j0_min_, j0_max_);
      // Image y grows downward; joint1 "up" is the max of its range.
      target_j1 = mapRange(ny, 0.0, 1.0, j1_max_, j1_min_);
      target_j2 = mapRange(area, min_object_area_, max_object_area_, j2_min_, j2_max_);
      target_j3 = j3_tracking_;
    } else {
      target_j0 = home_j0_;
      target_j1 = home_j1_;
      target_j2 = home_j2_;
      target_j3 = home_j3_;
    }

    std::vector<double> targets = {target_j0, target_j1, target_j2, target_j3};

    if (use_scurve_smoothing_) {
      // S-curve mode: only re-target when the move is significant enough to
      // matter. Small jitter is absorbed by the in-progress S-curve (or by
      // holding position when idle). Each S-curve has zero velocity at both
      // ends, so servos accelerate and decelerate gently instead of snapping.
      for (size_t i = 0; i < targets.size(); ++i) {
        if (std::abs(targets[i] - smoothers_[i].target()) > scurve_min_delta_rad_) {
          smoothers_[i].set_target(targets[i]);
        }
      }
    } else {
      // EMA mode (original behaviour): first-order smoothing with a hard
      // per-tick speed cap so large setpoint jumps don't swing the arm.
      double max_step = max_joint_speed_rad_s_ / control_rate_hz_;
      for (size_t i = 0; i < smoothed_.size(); ++i) {
        double step = smoothing_alpha_ * (targets[i] - smoothed_[i]);
        step = std::max(-max_step, std::min(max_step, step));
        smoothed_[i] += step;
      }
    }

    // Advance all smoothers by one control tick and write the result.
    double dt = 1.0 / control_rate_hz_;
    if (use_scurve_smoothing_) {
      for (size_t i = 0; i < smoothers_.size(); ++i) {
        smoothed_[i] = smoothers_[i].update(dt);
      }
    }

    publish();
  }

  void publish()
  {
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = smoothed_;
    cmd_pub_->publish(cmd_msg);

    builtin_interfaces::msg::Time stamp = now();
    builtin_interfaces::msg::Duration time_from_start =
      rclcpp::Duration::from_seconds(0.5);

    trajectory_msgs::msg::JointTrajectory arm_traj;
    arm_traj.header.stamp = stamp;
    arm_traj.joint_names = {"joint_0", "joint_1", "joint_2"};
    trajectory_msgs::msg::JointTrajectoryPoint arm_point;
    arm_point.positions = {smoothed_[0], smoothed_[1], smoothed_[2]};
    arm_point.time_from_start = time_from_start;
    arm_traj.points.push_back(arm_point);
    arm_traj_pub_->publish(arm_traj);

    trajectory_msgs::msg::JointTrajectory gripper_traj;
    gripper_traj.header.stamp = stamp;
    gripper_traj.joint_names = {"joint_3"};
    trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
    gripper_point.positions = {smoothed_[3]};
    gripper_point.time_from_start = time_from_start;
    gripper_traj.points.push_back(gripper_point);
    gripper_traj_pub_->publish(gripper_traj);
  }

  // Parameters
  int64_t image_width_, image_height_, object_lost_timeout_ms_;
  double min_object_area_, max_object_area_, min_confidence_;
  double smoothing_alpha_, deadband_px_;
  double max_joint_speed_rad_s_, control_rate_hz_;
  double j0_min_, j0_max_, j1_min_, j1_max_, j2_min_, j2_max_, j3_min_, j3_max_, j3_tracking_;
  double home_j0_, home_j1_, home_j2_, home_j3_;

  // State
  std::vector<double> smoothed_;
  std::vector<JointSmoother> smoothers_;   ///< Per-joint EMA or S-curve smoothers
  bool use_scurve_smoothing_;
  double scurve_duration_s_;
  double scurve_min_delta_rad_;
  std::array<double, 3> last_target_{};
  bool have_target_{false};
  bool have_seen_once_;
  std::string target_class_;
  rclcpp::Time last_seen_;

  // Interfaces
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_traj_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_class_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeArmVisualServoingNode>());
  rclcpp::shutdown();
  return 0;
}
