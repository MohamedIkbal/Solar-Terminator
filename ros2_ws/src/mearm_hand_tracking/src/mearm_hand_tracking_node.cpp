// MeArm hand-tracking teleop node.
//
// Captures frames from a USB camera, segments the hand by HSV skin color,
// finds its centroid / size / finger count, maps those to the MeArm's
// joint angles, and streams /arm_joint_commands (Float64MultiArray, 4 rad) to
// drive the PCA9685 servos on the STM32 directly over serial (no micro-ROS;
// see pca9685_multiple's servo_serial_bridge node).

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace
{
double mapRange(double value, double in_min, double in_max, double out_min, double out_max)
{
  double t = (value - in_min) / (in_max - in_min);
  t = std::clamp(t, 0.0, 1.0);
  return out_min + t * (out_max - out_min);
}
}  // namespace

class MeArmHandTrackingNode : public rclcpp::Node
{
public:
  MeArmHandTrackingNode()
  : Node("mearm_hand_tracking_node")
  {
    declare_parameter<int>("camera_index", 0);
    declare_parameter<int>("frame_width", 640);
    declare_parameter<int>("frame_height", 480);
    declare_parameter<double>("publish_rate_hz", 15.0);
    declare_parameter<double>("min_hand_area", 3000.0);
    declare_parameter<double>("max_hand_area", 45000.0);
    declare_parameter<double>("smoothing_alpha", 0.25);
    declare_parameter<bool>("show_debug_windows", true);
    declare_parameter<bool>("mirror_camera", true);
    declare_parameter<int>("hand_lost_timeout_ms", 800);
    declare_parameter<int>("h_min", 0);
    declare_parameter<int>("h_max", 20);
    declare_parameter<int>("s_min", 30);
    declare_parameter<int>("s_max", 150);
    declare_parameter<int>("v_min", 60);
    declare_parameter<int>("v_max", 255);

    camera_index_ = get_parameter("camera_index").as_int();
    frame_width_ = get_parameter("frame_width").as_int();
    frame_height_ = get_parameter("frame_height").as_int();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    min_hand_area_ = get_parameter("min_hand_area").as_double();
    max_hand_area_ = get_parameter("max_hand_area").as_double();
    smoothing_alpha_ = get_parameter("smoothing_alpha").as_double();
    show_debug_windows_ = get_parameter("show_debug_windows").as_bool();
    mirror_camera_ = get_parameter("mirror_camera").as_bool();
    hand_lost_timeout_ms_ = get_parameter("hand_lost_timeout_ms").as_int();
    h_min_ = get_parameter("h_min").as_int();
    h_max_ = get_parameter("h_max").as_int();
    s_min_ = get_parameter("s_min").as_int();
    s_max_ = get_parameter("s_max").as_int();
    v_min_ = get_parameter("v_min").as_int();
    v_max_ = get_parameter("v_max").as_int();

    cap_.open(camera_index_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      cap_.open(camera_index_);
    }
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Could not open camera index %d", camera_index_);
      throw std::runtime_error("camera open failed");
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);

    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_joint_commands", 10);

    // Start the arm at the midpoint of each joint's range.
    smoothed_joint0_ = 0.0;
    smoothed_joint1_ = (kJoint1Min + kJoint1Max) / 2.0;
    smoothed_joint2_ = (kJoint2Min + kJoint2Max) / 2.0;
    smoothed_joint3_ = kJoint3Max;  // start with gripper open
    last_seen_ = now();

    if (show_debug_windows_) {
      cv::namedWindow("Hand Tracking", cv::WINDOW_AUTOSIZE);
      cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);
      cv::namedWindow("Controls", cv::WINDOW_AUTOSIZE);
      cv::createTrackbar("H min", "Controls", &h_min_, 179);
      cv::createTrackbar("H max", "Controls", &h_max_, 179);
      cv::createTrackbar("S min", "Controls", &s_min_, 255);
      cv::createTrackbar("S max", "Controls", &s_max_, 255);
      cv::createTrackbar("V min", "Controls", &v_min_, 255);
      cv::createTrackbar("V max", "Controls", &v_max_, 255);
    }

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MeArmHandTrackingNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "mearm_hand_tracking_node started: camera %d, %dx%d, %.1f Hz. "
      "Move your hand: left/right -> base, up/down -> shoulder, "
      "closer/farther -> elbow, open/close fist -> gripper.",
      camera_index_, frame_width_, frame_height_, publish_rate_hz_);
  }

  ~MeArmHandTrackingNode() override
  {
    if (show_debug_windows_) {
      cv::destroyAllWindows();
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  static constexpr double kJoint0Min = -1.2, kJoint0Max = 1.2;
  static constexpr double kJoint1Min = -1.57, kJoint1Max = 1.0;
  static constexpr double kJoint2Min = 0.0, kJoint2Max = 1.57;
  static constexpr double kJoint3Min = 0.0, kJoint3Max = 0.42;

  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty camera frame");
      return;
    }
    if (mirror_camera_) {
      cv::flip(frame, frame, 1);
    }

    cv::Mat blurred, hsv, mask;
    cv::GaussianBlur(frame, blurred, cv::Size(7, 7), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(
      hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool hand_found = false;
    int fingers = 0;
    double area = 0.0;
    cv::Point2f centroid(0.f, 0.f);
    std::vector<cv::Point> hand_contour;

    if (!contours.empty()) {
      auto largest_it = std::max_element(
        contours.begin(), contours.end(),
        [](const auto & a, const auto & b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });
      area = cv::contourArea(*largest_it);
      if (area >= min_hand_area_) {
        hand_contour = *largest_it;
        hand_found = true;

        cv::Moments m = cv::moments(hand_contour);
        centroid = cv::Point2f(
          static_cast<float>(m.m10 / (m.m00 + 1e-6)),
          static_cast<float>(m.m01 / (m.m00 + 1e-6)));

        fingers = countFingers(hand_contour);
      }
    }

    if (hand_found) {
      last_seen_ = now();

      double nx = centroid.x / static_cast<double>(frame.cols);
      double ny = centroid.y / static_cast<double>(frame.rows);

      double target_j0 = mapRange(nx, 0.0, 1.0, kJoint0Min, kJoint0Max);
      double target_j1 = mapRange(ny, 0.0, 1.0, kJoint1Max, kJoint1Min);
      double target_j2 = mapRange(area, min_hand_area_, max_hand_area_, kJoint2Min, kJoint2Max);
      double openness = std::clamp((fingers - 1) / 4.0, 0.0, 1.0);
      double target_j3 = kJoint3Min + openness * (kJoint3Max - kJoint3Min);

      smoothed_joint0_ += smoothing_alpha_ * (target_j0 - smoothed_joint0_);
      smoothed_joint1_ += smoothing_alpha_ * (target_j1 - smoothed_joint1_);
      smoothed_joint2_ += smoothing_alpha_ * (target_j2 - smoothed_joint2_);
      smoothed_joint3_ += smoothing_alpha_ * (target_j3 - smoothed_joint3_);

      publishTrajectories();
    } else {
      auto since_seen = (now() - last_seen_).to_chrono<std::chrono::milliseconds>();
      if (since_seen.count() > hand_lost_timeout_ms_) {
        // Hand lost for too long: hold current pose, don't spam new goals.
      }
    }

    if (show_debug_windows_) {
      drawDebug(frame, mask, hand_contour, hand_found, fingers, area, centroid);
    }
  }

  int countFingers(const std::vector<cv::Point> & contour)
  {
    std::vector<int> hull_indices;
    cv::convexHull(contour, hull_indices, false, false);
    if (hull_indices.size() < 4) {
      return 1;
    }
    std::vector<cv::Vec4i> defects;
    try {
      cv::convexityDefects(contour, hull_indices, defects);
    } catch (const cv::Exception &) {
      return 1;
    }

    int finger_gaps = 0;
    const double min_depth = 18.0;  // pixels, filters out noise in the contour
    for (const auto & d : defects) {
      cv::Point start = contour[d[0]];
      cv::Point end = contour[d[1]];
      cv::Point far = contour[d[2]];
      double depth = d[3] / 256.0;
      if (depth < min_depth) {
        continue;
      }
      double a = cv::norm(start - far);
      double b = cv::norm(end - far);
      double c = cv::norm(start - end);
      double cos_angle = (a * a + b * b - c * c) / (2.0 * a * b + 1e-6);
      double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
      if (angle < CV_PI * 0.55) {  // valley between two extended fingers
        finger_gaps++;
      }
    }
    return std::min(finger_gaps + 1, 5);
  }

  void publishTrajectories()
  {
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = {
      smoothed_joint0_, smoothed_joint1_, smoothed_joint2_, smoothed_joint3_};
    cmd_pub_->publish(cmd_msg);
  }

  void drawDebug(
    cv::Mat & frame, const cv::Mat & mask, const std::vector<cv::Point> & hand_contour,
    bool hand_found, int fingers, double area, const cv::Point2f & centroid)
  {
    if (hand_found) {
      std::vector<std::vector<cv::Point>> draw_contours{hand_contour};
      cv::drawContours(frame, draw_contours, 0, cv::Scalar(0, 255, 0), 2);
      std::vector<cv::Point> hull_points;
      cv::convexHull(hand_contour, hull_points);
      std::vector<std::vector<cv::Point>> draw_hull{hull_points};
      cv::drawContours(frame, draw_hull, 0, cv::Scalar(255, 0, 0), 1);
      cv::circle(frame, centroid, 6, cv::Scalar(0, 0, 255), -1);
    }

    char buf[160];
    snprintf(
      buf, sizeof(buf), "hand:%s area:%.0f fingers:%d",
      hand_found ? "yes" : "no", area, fingers);
    cv::putText(
      frame, buf, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);
    snprintf(
      buf, sizeof(buf), "j0:%.2f j1:%.2f j2:%.2f j3:%.2f",
      smoothed_joint0_, smoothed_joint1_, smoothed_joint2_, smoothed_joint3_);
    cv::putText(
      frame, buf, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);

    cv::imshow("Hand Tracking", frame);
    cv::imshow("Mask", mask);
    cv::waitKey(1);
  }

  // Camera / processing
  cv::VideoCapture cap_;
  int camera_index_;
  int frame_width_;
  int frame_height_;
  double publish_rate_hz_;
  double min_hand_area_;
  double max_hand_area_;
  double smoothing_alpha_;
  bool show_debug_windows_;
  bool mirror_camera_;
  int hand_lost_timeout_ms_;
  int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;

  // Smoothed joint targets
  double smoothed_joint0_, smoothed_joint1_, smoothed_joint2_, smoothed_joint3_;
  rclcpp::Time last_seen_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MeArmHandTrackingNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mearm_hand_tracking_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
