// MeArm space-object spotter.
//
// Captures frames from a USB camera, localizes the most prominent object in
// the frame with classic OpenCV blob/contour detection (works for a bright
// object against a dark background -- debris, a meteor streak, a solar
// sail panel, the sunlit limb against the Moon's dark south pole -- or a
// dark object against a lighter background, e.g. a locust on a leaf),
// classifies the crop with an ONNX Runtime CNN into one of five classes,
// and streams /arm_joint_commands (Float64MultiArray, 4 rad) so the MeArm's
// PCA9685 servos (on the STM32) aim at whatever it found. Publishes a
// vision_msgs/Detection2D per detection so the result is inspectable / loggable
// independent of the arm.
// detection so the result is inspectable / loggable independent of the arm.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

namespace
{
double mapRange(double value, double in_min, double in_max, double out_min, double out_max)
{
  double t = (value - in_min) / (in_max - in_min);
  t = std::clamp(t, 0.0, 1.0);
  return out_min + t * (out_max - out_min);
}

std::vector<std::string> loadLabels(const std::string & path)
{
  std::vector<std::string> labels;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty()) {
      labels.push_back(line);
    }
  }
  return labels;
}
}  // namespace

class MeArmObjectDetectionNode : public rclcpp::Node
{
public:
  MeArmObjectDetectionNode()
  : Node("mearm_object_detection_node"),
    ort_env_(ORT_LOGGING_LEVEL_WARNING, "mearm_object_detection")
  {
    declare_parameter<int>("camera_index", 0);
    declare_parameter<int>("frame_width", 640);
    declare_parameter<int>("frame_height", 480);
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<std::string>("model_path", "");
    declare_parameter<std::string>("labels_path", "");
    declare_parameter<double>("min_object_area", 400.0);
    declare_parameter<double>("max_object_area", 150000.0);
    declare_parameter<double>("min_confidence", 0.0);
    declare_parameter<double>(
      "unknown_confidence_threshold", 0.5);
    declare_parameter<std::string>("unknown_image_dir", "");
    declare_parameter<int>("unknown_image_save_interval_ms", 1500);
    declare_parameter<double>("smoothing_alpha", 0.25);
    declare_parameter<bool>("show_debug_windows", true);
    declare_parameter<bool>("mirror_camera", false);
    declare_parameter<int>("object_lost_timeout_ms", 800);
    declare_parameter<int>("canny_threshold1", 50);
    declare_parameter<int>("canny_threshold2", 150);
    declare_parameter<double>("track_gate_px", 120.0);
    declare_parameter<double>("track_switch_margin", 1.3);
    // When a dedicated visual servoing node (mearm_visual_servoing) is
    // running, it is the one subscribing to /object_detection/detections
    // and driving /arm_joint_commands + the joint trajectory controllers.
    // Keep this false in that case so the two control loops don't publish
    // competing targets; set true to fall back to this node's own inline
    // centroid-to-joint control.
    declare_parameter<bool>("enable_builtin_servo", false);

    camera_index_ = get_parameter("camera_index").as_int();
    frame_width_ = get_parameter("frame_width").as_int();
    frame_height_ = get_parameter("frame_height").as_int();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    model_path_ = get_parameter("model_path").as_string();
    labels_path_ = get_parameter("labels_path").as_string();
    min_object_area_ = get_parameter("min_object_area").as_double();
    max_object_area_ = get_parameter("max_object_area").as_double();
    min_confidence_ = get_parameter("min_confidence").as_double();
    unknown_confidence_threshold_ = get_parameter("unknown_confidence_threshold").as_double();
    unknown_image_dir_ = get_parameter("unknown_image_dir").as_string();
    unknown_image_save_interval_ms_ = get_parameter("unknown_image_save_interval_ms").as_int();
    smoothing_alpha_ = get_parameter("smoothing_alpha").as_double();
    show_debug_windows_ = get_parameter("show_debug_windows").as_bool();
    mirror_camera_ = get_parameter("mirror_camera").as_bool();
    object_lost_timeout_ms_ = get_parameter("object_lost_timeout_ms").as_int();
    canny_threshold1_ = get_parameter("canny_threshold1").as_int();
    canny_threshold2_ = get_parameter("canny_threshold2").as_int();
    track_gate_px_ = get_parameter("track_gate_px").as_double();
    track_switch_margin_ = get_parameter("track_switch_margin").as_double();
    enable_builtin_servo_ = get_parameter("enable_builtin_servo").as_bool();

    if (model_path_.empty() || labels_path_.empty()) {
      RCLCPP_FATAL(get_logger(), "model_path and labels_path parameters are required");
      throw std::runtime_error("missing model/labels path");
    }
    labels_ = loadLabels(labels_path_);
    if (labels_.empty()) {
      RCLCPP_FATAL(get_logger(), "No labels loaded from %s", labels_path_.c_str());
      throw std::runtime_error("empty labels file");
    }

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    ort_session_ = std::make_unique<Ort::Session>(ort_env_, model_path_.c_str(), session_options);

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
    // Drives the same smoothed joint targets into the ros2_control
    // JointTrajectoryControllers (arm_controller: joint_0..2, gripper_controller:
    // joint_3) that gz_ros2_control hosts in Gazebo. Without these, the real
    // servo moves via /arm_joint_commands but the simulated arm in Gazebo/RViz
    // (which reads its state from these controllers) never follows along.
    arm_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    gripper_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/gripper_controller/joint_trajectory", 10);
    detection_pub_ = create_publisher<vision_msgs::msg::Detection2D>(
      "/object_detection/detections", 10);
    target_class_sub_ = create_subscription<std_msgs::msg::String>(
      "/object_detection/target_class", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        target_class_ = msg->data;
        RCLCPP_INFO(get_logger(), "Target class set to: '%s'", target_class_.c_str());
      });

    smoothed_joint0_ = 0.0;
    smoothed_joint1_ = (kJoint1Min + kJoint1Max) / 2.0;
    smoothed_joint2_ = (kJoint2Min + kJoint2Max) / 2.0;
    smoothed_joint3_ = (kJoint3Min + kJoint3Max) / 2.0;  // gripper half-closed while aiming
    last_seen_ = now();
    last_unknown_save_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    // Unknown crops get written here so they're kept separate from the
    // labeled training_images/<class> folders and can be reviewed/sorted
    // later for retraining.
    if (!unknown_image_dir_.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(unknown_image_dir_, ec);
      if (ec) {
        RCLCPP_WARN(
          get_logger(), "Could not create unknown_image_dir '%s': %s",
          unknown_image_dir_.c_str(), ec.message().c_str());
        unknown_image_dir_.clear();
      } else {
        RCLCPP_INFO(
          get_logger(), "Saving unknown-class crops to %s", unknown_image_dir_.c_str());
      }
    }

    if (show_debug_windows_) {
      cv::namedWindow("Object Spotter", cv::WINDOW_AUTOSIZE);
      cv::namedWindow("Edges", cv::WINDOW_AUTOSIZE);
    }

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MeArmObjectDetectionNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "mearm_object_detection_node started: camera %d, %dx%d, %.1f Hz, %zu classes "
      "loaded from model %s",
      camera_index_, frame_width_, frame_height_, publish_rate_hz_, labels_.size(),
      model_path_.c_str());
  }

  ~MeArmObjectDetectionNode() override
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
  static constexpr int kModelSize = 64;

  // Original/home pose: centered pan, mid-range lift/reach, gripper
  // half-closed. The arm returns here whenever nothing is confidently
  // identified (unknown class) or no object has been seen recently.
  static constexpr double kHomeJoint0 = 0.0;
  static constexpr double kHomeJoint1 = (kJoint1Min + kJoint1Max) / 2.0;
  static constexpr double kHomeJoint2 = (kJoint2Min + kJoint2Max) / 2.0;
  static constexpr double kHomeJoint3 = (kJoint3Min + kJoint3Max) / 2.0;
  static constexpr const char * kUnknownLabel = "unknown";

  struct Candidate
  {
    cv::Rect box;
    double area {0.0};
    cv::Point2f centroid;
  };

  bool has_track_ {false};
  cv::Point2f tracked_centroid_ {};
  double tracked_area_ {0.0};
  cv::Point2f smoothed_centroid_ {};
  cv::Rect2f smoothed_box_ {};

  bool findCandidate(const cv::Mat & frame, cv::Mat & edges_out, Candidate & out)
  {
    cv::Mat gray, blurred, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edges, canny_threshold1_, canny_threshold2_);
    cv::dilate(edges, edges, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    edges_out = edges;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
      return false;
    }

    std::vector<Candidate> candidates;
    for (const auto & contour : contours) {
      double area = cv::contourArea(contour);
      if (area < min_object_area_ || area > max_object_area_) {
        continue;
      }
      Candidate cand;
      cand.box = cv::boundingRect(contour);
      cand.area = area;
      cv::Moments m = cv::moments(contour);
      cand.centroid = cv::Point2f(
        static_cast<float>(m.m10 / (m.m00 + 1e-6)),
        static_cast<float>(m.m01 / (m.m00 + 1e-6)));
      candidates.push_back(cand);
    }
    if (candidates.empty()) {
      return false;
    }

    auto largest_it = std::max_element(
      candidates.begin(), candidates.end(),
      [](const Candidate & a, const Candidate & b) {return a.area < b.area;});

    const Candidate * selected = nullptr;
    if (has_track_) {
      auto nearest_it = std::min_element(
        candidates.begin(), candidates.end(),
        [this](const Candidate & a, const Candidate & b) {
          return cv::norm(a.centroid - tracked_centroid_) < cv::norm(b.centroid - tracked_centroid_);
        });
      double nearest_dist = cv::norm(nearest_it->centroid - tracked_centroid_);
      if (nearest_dist <= track_gate_px_) {
        const Candidate * bigger = nullptr;
        for (const auto & cand : candidates) {
          if (&cand == &(*nearest_it)) {
            continue;
          }
          double dist = cv::norm(cand.centroid - tracked_centroid_);
          if (dist <= track_gate_px_ && cand.area >= track_switch_margin_ * nearest_it->area) {
            if (bigger == nullptr || cand.area > bigger->area) {
              bigger = &cand;
            }
          }
        }
        selected = (bigger != nullptr) ? bigger : &(*nearest_it);
      } else {
        selected = &(*largest_it);
      }
    } else {
      selected = &(*largest_it);
    }

    bool is_first = !has_track_;
    has_track_ = true;
    tracked_centroid_ = selected->centroid;
    tracked_area_ = selected->area;

    if (is_first) {
      smoothed_centroid_ = selected->centroid;
      smoothed_box_ = cv::Rect2f(selected->box);
    } else {
      smoothed_centroid_.x += smoothing_alpha_ * (selected->centroid.x - smoothed_centroid_.x);
      smoothed_centroid_.y += smoothing_alpha_ * (selected->centroid.y - smoothed_centroid_.y);
      smoothed_box_.x += smoothing_alpha_ * (selected->box.x - smoothed_box_.x);
      smoothed_box_.y += smoothing_alpha_ * (selected->box.y - smoothed_box_.y);
      smoothed_box_.width += smoothing_alpha_ * (selected->box.width - smoothed_box_.width);
      smoothed_box_.height += smoothing_alpha_ * (selected->box.height - smoothed_box_.height);
    }

    out.centroid = smoothed_centroid_;
    out.box = cv::Rect(smoothed_box_);
    out.area = selected->area;
    return true;
  }

  // Crops, resizes to the model's 64x64 NCHW input and runs the ONNX
  // classifier. Returns the best class index and its softmax score.
  std::pair<int, float> classify(const cv::Mat & frame, const cv::Rect & box)
  {
    cv::Rect clamped = box & cv::Rect(0, 0, frame.cols, frame.rows);
    cv::Mat crop = frame(clamped);
    cv::Mat resized, rgb;
    cv::resize(crop, resized, cv::Size(kModelSize, kModelSize));
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

    std::vector<float> input_tensor_values(3 * kModelSize * kModelSize);
    std::vector<cv::Mat> channels(3);
    cv::split(rgb, channels);
    for (int c = 0; c < 3; ++c) {
      std::memcpy(
        input_tensor_values.data() + c * kModelSize * kModelSize,
        channels[c].data, kModelSize * kModelSize * sizeof(float));
    }

    std::array<int64_t, 4> input_shape {1, 3, kModelSize, kModelSize};
    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info, input_tensor_values.data(), input_tensor_values.size(),
      input_shape.data(), input_shape.size());

    const char * input_names[] = {"input"};
    const char * output_names[] = {"output"};
    auto output_tensors = ort_session_->Run(
      Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    const float * scores = output_tensors.front().GetTensorData<float>();
    int best_idx = 0;
    float best_score = scores[0];
    for (size_t i = 1; i < labels_.size(); ++i) {
      if (scores[i] > best_score) {
        best_score = scores[i];
        best_idx = static_cast<int>(i);
      }
    }
    return {best_idx, best_score};
  }

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

    cv::Mat edges;
    Candidate candidate;
    bool found = findCandidate(frame, edges, candidate);
    std::string label = "none";
    float score = 0.0f;

    if (found) {
      auto [idx, conf] = classify(frame, candidate.box);
      if (conf >= min_confidence_) {
        score = conf;
        last_seen_ = now();

        if (conf < unknown_confidence_threshold_) {
          // Something is there, but the classifier isn't confident enough
          // to call it one of the known classes: report it as unknown and
          // send the arm back to its original resting pose instead of
          // aiming at it.
          label = kUnknownLabel;
          publishDetection(candidate, label, score);
          saveUnknownCrop(frame, candidate);
          if (enable_builtin_servo_) {
            goHome();
          }
        } else {
          label = labels_[idx];
          publishDetection(candidate, label, score);

          // Only steer the arm when the detected class matches the class
          // picked on the Node-RED dashboard (an empty target means "any
          // class", kept for backward compatibility with older flows).
          bool matches_target = target_class_.empty() || label == target_class_;
          if (enable_builtin_servo_ && matches_target) {
            double nx = candidate.centroid.x / static_cast<double>(frame.cols);
            double ny = candidate.centroid.y / static_cast<double>(frame.rows);

            double target_j0 = mapRange(nx, 0.0, 1.0, kJoint0Min, kJoint0Max);
            double target_j1 = mapRange(ny, 0.0, 1.0, kJoint1Max, kJoint1Min);
            double target_j2 = mapRange(
              candidate.area, min_object_area_, max_object_area_, kJoint2Min, kJoint2Max);

            smoothed_joint0_ += smoothing_alpha_ * (target_j0 - smoothed_joint0_);
            smoothed_joint1_ += smoothing_alpha_ * (target_j1 - smoothed_joint1_);
            smoothed_joint2_ += smoothing_alpha_ * (target_j2 - smoothed_joint2_);

            publishTrajectories();
          }
        }
      } else {
        found = false;
      }
    }

    if (!found) {
      auto since_seen = (now() - last_seen_).to_chrono<std::chrono::milliseconds>();
      if (enable_builtin_servo_ && since_seen.count() > object_lost_timeout_ms_) {
        // Object lost for too long: ease the arm back to its original
        // resting pose rather than leaving it aimed at empty space.
        goHome();
      }
    }

    if (show_debug_windows_) {
      drawDebug(frame, edges, found, candidate, label, score);
    }
  }

  // Crops the current candidate and writes it into unknown_image_dir_ so
  // low-confidence detections are kept separate from the labeled
  // training_images/<class> folders. Throttled to
  // unknown_image_save_interval_ms_ so a lingering unidentified object
  // doesn't flood the disk with near-duplicate frames.
  void saveUnknownCrop(const cv::Mat & frame, const Candidate & c)
  {
    if (unknown_image_dir_.empty()) {
      return;
    }
    auto since_last_save = (now() - last_unknown_save_).to_chrono<std::chrono::milliseconds>();
    if (since_last_save.count() < unknown_image_save_interval_ms_) {
      return;
    }
    cv::Rect clamped = c.box & cv::Rect(0, 0, frame.cols, frame.rows);
    if (clamped.width <= 0 || clamped.height <= 0) {
      return;
    }
    cv::Mat crop = frame(clamped);

    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream name;
    name << "unknown_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << ".jpg";
    std::filesystem::path out_path = std::filesystem::path(unknown_image_dir_) / name.str();

    if (cv::imwrite(out_path.string(), crop)) {
      last_unknown_save_ = now();
      RCLCPP_INFO(get_logger(), "Saved unknown crop: %s", out_path.string().c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to save unknown crop to %s", out_path.string().c_str());
    }
  }

  // Smoothly eases the arm/gripper back toward the home pose and publishes
  // it. Called whenever the classifier can't confidently identify the
  // object (unknown) or no object has been seen for object_lost_timeout_ms_.
  void goHome()
  {
    smoothed_joint0_ += smoothing_alpha_ * (kHomeJoint0 - smoothed_joint0_);
    smoothed_joint1_ += smoothing_alpha_ * (kHomeJoint1 - smoothed_joint1_);
    smoothed_joint2_ += smoothing_alpha_ * (kHomeJoint2 - smoothed_joint2_);
    smoothed_joint3_ += smoothing_alpha_ * (kHomeJoint3 - smoothed_joint3_);
    publishTrajectories();
  }

  void publishTrajectories()
  {
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = {
      smoothed_joint0_, smoothed_joint1_, smoothed_joint2_, smoothed_joint3_};
    cmd_pub_->publish(cmd_msg);

    // Mirror the same targets to the ros2_control JointTrajectoryControllers
    // so Gazebo's simulated arm and RViz's displayed arm follow the same
    // motion as the real servo driven via /arm_joint_commands above.
    rclcpp::Duration time_from_start = rclcpp::Duration::from_seconds(0.2);

    trajectory_msgs::msg::JointTrajectory arm_traj;
    arm_traj.header.stamp = now();
    arm_traj.joint_names = {"joint_0", "joint_1", "joint_2"};
    trajectory_msgs::msg::JointTrajectoryPoint arm_point;
    arm_point.positions = {smoothed_joint0_, smoothed_joint1_, smoothed_joint2_};
    arm_point.time_from_start = time_from_start;
    arm_traj.points.push_back(arm_point);
    arm_traj_pub_->publish(arm_traj);

    trajectory_msgs::msg::JointTrajectory gripper_traj;
    gripper_traj.header.stamp = now();
    gripper_traj.joint_names = {"joint_3"};
    trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
    gripper_point.positions = {smoothed_joint3_};
    gripper_point.time_from_start = time_from_start;
    gripper_traj.points.push_back(gripper_point);
    gripper_traj_pub_->publish(gripper_traj);
  }

  void publishDetection(const Candidate & c, const std::string & label, float score)
  {
    vision_msgs::msg::Detection2D msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_link";
    msg.id = label;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = label;
    hyp.hypothesis.score = score;
    msg.results.push_back(hyp);

    msg.bbox.center.position.x = c.centroid.x;
    msg.bbox.center.position.y = c.centroid.y;
    msg.bbox.size_x = c.box.width;
    msg.bbox.size_y = c.box.height;

    detection_pub_->publish(msg);
  }

  void drawDebug(
    cv::Mat & frame, const cv::Mat & edges, bool found, const Candidate & c,
    const std::string & label, float score)
  {
    if (found) {
      cv::rectangle(frame, c.box, cv::Scalar(0, 255, 0), 2);
      cv::circle(frame, c.centroid, 5, cv::Scalar(0, 0, 255), -1);
    }
    char buf[160];
    snprintf(
      buf, sizeof(buf), "class:%s score:%.2f area:%.0f target:%s",
      label.c_str(), score, c.area, target_class_.empty() ? "any" : target_class_.c_str());
    cv::putText(
      frame, buf, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);
    snprintf(
      buf, sizeof(buf), "j0:%.2f j1:%.2f j2:%.2f",
      smoothed_joint0_, smoothed_joint1_, smoothed_joint2_);
    cv::putText(
      frame, buf, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);

    cv::imshow("Object Spotter", frame);
    cv::imshow("Edges", edges);
    cv::waitKey(1);
  }

  // Camera / processing
  cv::VideoCapture cap_;
  int camera_index_;
  int frame_width_;
  int frame_height_;
  double track_gate_px_;
  double track_switch_margin_;
  double publish_rate_hz_;
  std::string model_path_;
  std::string labels_path_;
  double min_object_area_;
  double max_object_area_;
  double min_confidence_;
  double unknown_confidence_threshold_;
  std::string unknown_image_dir_;
  int unknown_image_save_interval_ms_;
  double smoothing_alpha_;
  bool show_debug_windows_;

  bool mirror_camera_;
  int object_lost_timeout_ms_;
  int canny_threshold1_, canny_threshold2_;
  bool enable_builtin_servo_;

  std::vector<std::string> labels_;
  std::string target_class_;  // "" = track the most prominent object of any class
  Ort::Env ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;

  // Smoothed joint targets
  double smoothed_joint0_, smoothed_joint1_, smoothed_joint2_, smoothed_joint3_;
  rclcpp::Time last_seen_;
  rclcpp::Time last_unknown_save_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_traj_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr detection_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_class_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MeArmObjectDetectionNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mearm_object_detection_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
