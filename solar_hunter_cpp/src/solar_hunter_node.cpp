#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include <solar_hunter_cpp/detector.hpp>
#include <solar_hunter_cpp/reflector_angle.hpp>
#include <solar_hunter_cpp/targets.hpp>

using namespace std::chrono_literals;

namespace solar_hunter {

// Index of the pan and tilt joints inside the MeArm's joint vector.
// Adjust these to match the real arm's joint order (check `ros2 topic
// echo /joint_states` -> name[] array index).
constexpr int kPanJointIndex = 0;
constexpr int kTiltJointIndex = 1;
constexpr int kDefaultNumJoints = 4;
constexpr double kDetectionConfidenceThreshold = 0.5;

class SolarHunterNode : public rclcpp::Node {
public:
  SolarHunterNode() : Node("solar_hunter") {
    // ----- parameters -----
    this->declare_parameter<int>("camera_index", 4);
    this->declare_parameter<std::string>("target", "meteors");
    this->declare_parameter<int>("pan_joint_index", kPanJointIndex);
    this->declare_parameter<int>("tilt_joint_index", kTiltJointIndex);
    this->declare_parameter<int>("num_joints", kDefaultNumJoints);
    this->declare_parameter<double>("publish_rate_hz", 10.0);
    this->declare_parameter<bool>("show_debug_windows", true);
    this->declare_parameter<bool>("mirror_camera", false);
    this->declare_parameter<bool>("auto_target", true);
    this->declare_parameter<bool>("target_is_manual", false);

    this->get_parameter("camera_index", camera_index_);
    this->get_parameter("target", target_name_);
    this->get_parameter("pan_joint_index", pan_joint_index_);
    this->get_parameter("tilt_joint_index", tilt_joint_index_);
    this->get_parameter("num_joints", num_joints_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("show_debug_windows", show_debug_windows_);
    this->get_parameter("mirror_camera", mirror_camera_);
    this->get_parameter("auto_target", auto_target_);
    this->get_parameter("target_is_manual", target_is_manual_);

    // ----- base aim from target -----
    try {
      const Target& t = Targets::get_target(target_name_);
      base_aim_ = mirror_axis_to_servo_angles(t.direction_vector());
      RCLCPP_INFO(this->get_logger(),
                  "Target '%s' (az=%.1f el=%.1f) -> base pan=%.2f deg tilt=%.2f deg%s",
                  t.name.c_str(), t.azimuth_deg, t.elevation_deg, base_aim_.pan_deg,
                  base_aim_.tilt_deg, base_aim_.clipped ? " [CLIPPED]" : "");
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load target '%s': %s", target_name_.c_str(),
                   e.what());
      rclcpp::shutdown();
      return;
    }

    // ----- ROS 2 pub/sub -----
    joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_joint_commands", 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (!msg->position.empty()) {
            last_positions_.assign(msg->position.begin(), msg->position.end());
          }
        });
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2D>(
        "/object_detection/detections", 10,
        [this](const vision_msgs::msg::Detection2D::SharedPtr msg) {
          const std::string label = msg->id;
          const double score = msg->results.empty() ? 0.0 : msg->results[0].hypothesis.score;
          if (!label.empty() && label != "unknown" && label != "none" &&
              score >= kDetectionConfidenceThreshold) {
            try {
              const Target& t = Targets::get_target(label);
              if (t.name != detected_target_name_) {
                RCLCPP_INFO(this->get_logger(),
                            "[object_detection] classified target: %s (score=%.2f)",
                            t.name.c_str(), score);
                detected_target_name_ = t.name;
              }
            } catch (const std::exception&) {
              // classifier emitted a label with no matching target; ignore.
            }
          }
        });

    // ----- camera -----
    if (camera_index_ >= 0) {
      cap_.open(camera_index_, cv::CAP_V4L2);
      if (!cap_.isOpened()) {
        cap_.open(camera_index_);
      }
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open camera index %d; running aim-only.",
                     camera_index_);
        camera_index_ = -1;
      } else {
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        RCLCPP_INFO(this->get_logger(), "Opened camera index %d.", camera_index_);
      }
    }

    // ----- timer -----
    const auto period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms), [this]() { this->timer_callback(); });

    RCLCPP_INFO(this->get_logger(), "solar_hunter node ready (target=%s, camera=%d)",
                target_name_.c_str(), camera_index_);
  }

  ~SolarHunterNode() {
    if (cap_.isOpened()) cap_.release();
    if (show_debug_windows_) cv::destroyAllWindows();
  }

private:
  // If the object-detection classifier (stage 1) has locked onto a new
  // target class since our last aim, re-point the base aim at it before
  // stage 2 (light/sun detection) refines the pointing further.
  void maybe_retarget() {
    if (auto_target_ && !target_is_manual_ && !detected_target_name_.empty() &&
        detected_target_name_ != target_name_) {
      try {
        const Target& t = Targets::get_target(detected_target_name_);
        base_aim_ = mirror_axis_to_servo_angles(t.direction_vector());
        target_name_ = detected_target_name_;
        RCLCPP_INFO(this->get_logger(),
                    "[object_detection] retargeting to '%s' (az=%.1f el=%.1f) -> base pan=%.2f deg tilt=%.2f deg%s",
                    t.name.c_str(), t.azimuth_deg, t.elevation_deg, base_aim_.pan_deg,
                    base_aim_.tilt_deg, base_aim_.clipped ? " [CLIPPED]" : "");
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to retarget to '%s': %s",
                    detected_target_name_.c_str(), e.what());
      }
    }
  }

  void timer_callback() {
    // ----- stage 1: object detection may have classified a new target -----
    maybe_retarget();

    double pan_deg = base_aim_.pan_deg;
    double tilt_deg = base_aim_.tilt_deg;
    bool blob_found = false;

    if (camera_index_ >= 0 && cap_.isOpened()) {
      cv::Mat frame;
      if (!cap_.read(frame)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Failed to read camera frame.");
        return;
      }
      if (mirror_camera_) cv::flip(frame, frame, 1);

      const BlobResult blob = detect_light_blob(frame);
      if (blob.found) {
        blob_found = true;
        const auto [pan_corr, tilt_corr] = focus_alignment_correction(blob);
        pan_deg = base_aim_.pan_deg + pan_corr;
        tilt_deg = base_aim_.tilt_deg + tilt_corr;
      }

      if (show_debug_windows_) {
        cv::Mat debug = draw_debug_overlay(frame, blob);
        char buf[128];
        std::snprintf(buf, sizeof(buf), "pan=%.1f tilt=%.1f %s", pan_deg, tilt_deg,
                      blob_found ? "blob" : "no-blob");
        cv::putText(debug, buf, cv::Point(10, debug.rows - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
        cv::imshow("solar-hunter", debug);
        if (cv::waitKey(1) == 27) {  // ESC
          rclcpp::shutdown();
        }
      }
    }

    publish_pan_tilt(pan_deg, tilt_deg);
  }

  void publish_pan_tilt(double pan_deg, double tilt_deg) {
    std::vector<double> positions = last_positions_;
    const int needed = std::max(pan_joint_index_, tilt_joint_index_) + 1;
    if (static_cast<int>(positions.size()) < needed) {
      positions.resize(needed, 0.0);
    }

    positions[pan_joint_index_] = deg2rad(pan_deg);
    positions[tilt_joint_index_] = deg2rad(tilt_deg);

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(positions.begin(), positions.end());
    joint_cmd_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "published /arm_joint_commands: pan=%.2f deg tilt=%.2f deg", pan_deg,
                         tilt_deg);
  }

  static double deg2rad(double d) { return d * M_PI / 180.0; }

  // ----- params -----
  int camera_index_;
  std::string target_name_;
  int pan_joint_index_;
  int tilt_joint_index_;
  int num_joints_;
  double publish_rate_hz_;
  bool show_debug_windows_;
  bool auto_target_;
  bool target_is_manual_;
  bool mirror_camera_;

  // ----- state -----
  AimResult base_aim_;
  std::vector<double> last_positions_;
  std::string detected_target_name_;
  cv::VideoCapture cap_;

  // ----- ROS 2 -----
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

}  // namespace solar_hunter

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<solar_hunter::SolarHunterNode>());
  rclcpp::shutdown();
  return 0;
}