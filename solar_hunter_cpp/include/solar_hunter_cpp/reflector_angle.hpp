#ifndef SOLAR_HUNTER_CPP_REFLECTOR_ANGLE_HPP
#define SOLAR_HUNTER_CPP_REFLECTOR_ANGLE_HPP

#include "detector.hpp"

#include <array>
#include <utility>

namespace solar_hunter {

// Parabolic reflector focal length (m).
constexpr double FOCUS_DISTANCE_M = 1.0;

// Approximate camera horizontal/vertical field of view (deg). Used only to
// convert pixel error into an angular error for the focus-centering feedback
// loop — adjust to the real webcam/lens in use.
constexpr double CAMERA_HFOV_DEG = 60.0;
constexpr double CAMERA_VFOV_DEG = 45.0;

// Proportional gain for the focus-centering correction (dimensionless,
// applied to the raw angular pixel error).
constexpr double CENTERING_GAIN = 0.5;

// MeArm gripper servo limits (degrees) — adjust to the actual rig.
constexpr double PAN_SERVO_RANGE_MIN_DEG = -90.0;
constexpr double PAN_SERVO_RANGE_MAX_DEG = 90.0;
constexpr double TILT_SERVO_RANGE_MIN_DEG = 0.0;
constexpr double TILT_SERVO_RANGE_MAX_DEG = 90.0;

struct AimResult {
  double pan_deg;
  double tilt_deg;
  double azimuth_deg;
  double elevation_deg;
  bool clipped;
};

// Convert a unit aim-direction vector (x fwd, y left, z up) into MeArm
// gripper pan/tilt servo angles that orient the reflector's axis along that
// direction.
AimResult mirror_axis_to_servo_angles(const std::array<double, 3>& direction_vector);

// General law-of-reflection solve: given a unit incident ray direction
// (pointing INTO the mirror) and a desired outgoing unit direction (pointing
// AWAY from the mirror toward the target), return the mirror surface normal
// that makes that reflection happen. Used when the source is not exactly at
// the focus and a direct normal solve is needed.
std::array<double, 3> reflect_incident_to_target(const std::array<double, 3>& incident_dir,
                                                  const std::array<double, 3>& target_dir);

// Given a detected blob, return (pan_correction_deg, tilt_correction_deg) to
// re-center it in the camera frame, i.e. keep it sitting on the reflector's
// focal point. Returns (0.0, 0.0) if no blob was found or it is already
// centered.
std::pair<double, double> focus_alignment_correction(const BlobResult& blob);

// Small helpers.
std::array<double, 3> normalize_vec3(const std::array<double, 3>& v);
double deg2rad(double deg);
double rad2deg(double rad);

}  // namespace solar_hunter

#endif  // SOLAR_HUNTER_CPP_REFLECTOR_ANGLE_HPP
