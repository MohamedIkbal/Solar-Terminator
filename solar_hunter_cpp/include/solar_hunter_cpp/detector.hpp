#ifndef SOLAR_HUNTER_CPP_DETECTOR_HPP
#define SOLAR_HUNTER_CPP_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <string>

namespace solar_hunter {

// Result of detecting a bright circular light source (white LED or yellow
// sunlight glare) in a camera frame.
struct BlobResult {
  bool found = false;
  double cx = 0.0;          // pixel x of blob center
  double cy = 0.0;          // pixel y of blob center
  double radius = 0.0;      // pixel radius
  std::string color_label;  // "white_led" or "yellow_sun"
  int frame_width = 0;
  int frame_height = 0;
};

// HSV thresholds — tuned for typical webcam exposure; adjust per camera.
extern const cv::Scalar WHITE_LOWER;   // H,S,V  = 0,  0, 220
extern const cv::Scalar WHITE_UPPER;   // H,S,V  = 180,60,255
extern const cv::Scalar YELLOW_LOWER;  // H,S,V  = 18, 80,180
extern const cv::Scalar YELLOW_UPPER;  // H,S,V  = 35,255,255

constexpr int MIN_BLOB_RADIUS_PX = 2;
constexpr double MIN_CIRCULARITY = 0.35;

// Detect the dominant white-LED or yellow-sunlight circular blob.
BlobResult detect_light_blob(const cv::Mat& frame_bgr);

// Draw the detected blob on a copy of the frame for visual debugging.
cv::Mat draw_debug_overlay(const cv::Mat& frame_bgr, const BlobResult& result);

}  // namespace solar_hunter

#endif  // SOLAR_HUNTER_CPP_DETECTOR_HPP
