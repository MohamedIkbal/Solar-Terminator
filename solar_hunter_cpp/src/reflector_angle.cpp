#include <solar_hunter_cpp/reflector_angle.hpp>

#include <cmath>

namespace solar_hunter {

std::array<double, 3> normalize_vec3(const std::array<double, 3>& v) {
  const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n < 1e-12) return {0.0, 0.0, 1.0};
  return {v[0] / n, v[1] / n, v[2] / n};
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

AimResult mirror_axis_to_servo_angles(const std::array<double, 3>& direction_vector) {
  const std::array<double, 3> v = normalize_vec3(direction_vector);
  const double x = v[0];
  const double y = v[1];
  const double z = v[2];

  const double azimuth_deg = rad2deg(std::atan2(y, x));
  const double elevation_deg = rad2deg(std::atan2(z, std::hypot(x, y)));

  // MeArm pan servo rotates the base about Z -> directly tracks azimuth.
  // MeArm tilt servo (wrist) raises the gripper -> directly tracks elevation.
  auto clip = [](double value, double lo, double hi) -> std::pair<double, bool> {
    bool clipped = value < lo || value > hi;
    double v = value;
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return {v, clipped};
  };

  auto [pan_deg, pan_clipped] = clip(azimuth_deg, PAN_SERVO_RANGE_MIN_DEG, PAN_SERVO_RANGE_MAX_DEG);
  auto [tilt_deg, tilt_clipped] =
      clip(elevation_deg, TILT_SERVO_RANGE_MIN_DEG, TILT_SERVO_RANGE_MAX_DEG);

  return AimResult{pan_deg, tilt_deg, azimuth_deg, elevation_deg,
                   pan_clipped || tilt_clipped};
}

std::array<double, 3> reflect_incident_to_target(const std::array<double, 3>& incident_dir,
                                                  const std::array<double, 3>& target_dir) {
  const std::array<double, 3> d_in = normalize_vec3(incident_dir);
  const std::array<double, 3> d_out = normalize_vec3(target_dir);

  std::array<double, 3> normal = {d_out[0] - d_in[0], d_out[1] - d_in[1], d_out[2] - d_in[2]};
  const double norm_len = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                                    normal[2] * normal[2]);
  if (norm_len < 1e-9) {
    // Incident and target directions coincide -> any normal perpendicular to
    // the ray works; default to "straight up".
    return {0.0, 0.0, 1.0};
  }
  return {normal[0] / norm_len, normal[1] / norm_len, normal[2] / norm_len};
}

std::pair<double, double> focus_alignment_correction(const BlobResult& blob) {
  if (!blob.found || blob.frame_width == 0 || blob.frame_height == 0) {
    return {0.0, 0.0};
  }

  const double half_w = blob.frame_width / 2.0;
  const double half_h = blob.frame_height / 2.0;

  // -1..1, + = blob right of center / below center.
  const double err_x = (blob.cx - half_w) / half_w;
  const double err_y = (blob.cy - half_h) / half_h;

  const double pan_correction_deg =
      -CENTERING_GAIN * (err_x * CAMERA_HFOV_DEG / 2.0);
  const double tilt_correction_deg =
      -CENTERING_GAIN * (err_y * CAMERA_VFOV_DEG / 2.0);

  return {pan_correction_deg, tilt_correction_deg};
}

}  // namespace solar_hunter
