#include <solar_hunter_cpp/detector.hpp>

#include <algorithm>

namespace solar_hunter {

const cv::Scalar WHITE_LOWER(0, 0, 220);     // H,S,V
const cv::Scalar WHITE_UPPER(180, 60, 255);  // H,S,V
const cv::Scalar YELLOW_LOWER(18, 80, 180);  // H,S,V
const cv::Scalar YELLOW_UPPER(35, 255, 255); // H,S,V

namespace {

// Find the largest, roundest contour in a binary mask.
// Returns (cx, cy, radius, circularity) or an empty optional if none qualify.
std::optional<std::tuple<double, double, double, double>> best_circle_from_mask(
    const cv::Mat& mask) {
  cv::Mat cleaned = mask;
  cv::morphologyEx(cleaned, cleaned, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8U));
  cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE, cv::Mat::ones(5, 5, CV_8U));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(cleaned, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::optional<std::tuple<double, double, double, double>> best;
  double best_score = -1.0;

  for (const auto& c : contours) {
    const double area = cv::contourArea(c);
    if (area <= 0.0) continue;

    cv::Point2f center;
    float radius = 0.f;
    cv::minEnclosingCircle(c, center, radius);
    if (radius < MIN_BLOB_RADIUS_PX) continue;

    const double circularity = area / (M_PI * radius * radius + 1e-6);
    if (circularity < MIN_CIRCULARITY) continue;

    // Rank by roundness first: a true point light source is compact and
    // circular, unlike broad glare patches.
    if (circularity > best_score) {
      best_score = circularity;
      best = std::make_tuple(static_cast<double>(center.x), static_cast<double>(center.y),
                             static_cast<double>(radius), circularity);
    }
  }
  return best;
}

}  // namespace

BlobResult detect_light_blob(const cv::Mat& frame_bgr) {
  BlobResult result;
  result.frame_width = frame_bgr.cols;
  result.frame_height = frame_bgr.rows;

  cv::Mat hsv;
  cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat white_mask, yellow_mask;
  cv::inRange(hsv, WHITE_LOWER, WHITE_UPPER, white_mask);
  cv::inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, yellow_mask);

  struct Candidate {
    std::string label;
    double cx, cy, radius, circ;
  };
  std::vector<Candidate> candidates;

  auto white_hit = best_circle_from_mask(white_mask);
  if (white_hit.has_value()) {
    const auto& [cx, cy, radius, circ] = *white_hit;
    candidates.push_back({"white_led", cx, cy, radius, circ});
  }

  auto yellow_hit = best_circle_from_mask(yellow_mask);
  if (yellow_hit.has_value()) {
    const auto& [cx, cy, radius, circ] = *yellow_hit;
    candidates.push_back({"yellow_sun", cx, cy, radius, circ});
  }

  if (candidates.empty()) {
    return result;
  }

  // Prefer the roundest (most point-like) of the two candidates.
  const auto& best =
      *std::max_element(candidates.begin(), candidates.end(),
                        [](const Candidate& a, const Candidate& b) {
                          return a.circ < b.circ;
                        });

  result.found = true;
  result.cx = best.cx;
  result.cy = best.cy;
  result.radius = best.radius;
  result.color_label = best.label;
  return result;
}

cv::Mat draw_debug_overlay(const cv::Mat& frame_bgr, const BlobResult& result) {
  cv::Mat out = frame_bgr.clone();

  const int display_radius = std::max(static_cast<int>(result.radius), 10);
  const cv::Point center(static_cast<int>(result.cx), static_cast<int>(result.cy));

  if (result.found) {
    cv::circle(out, center, display_radius, cv::Scalar(0, 255, 0), 5);
    cv::circle(out, center, 2, cv::Scalar(0, 0, 255), 3);

    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s r=%.1fpx", result.color_label.c_str(), result.radius);
    cv::putText(out, buf, cv::Point(center.x + 10, center.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  } else {
    cv::putText(out, "no light blob detected", cv::Point(10, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  }
  return out;
}

}  // namespace solar_hunter
