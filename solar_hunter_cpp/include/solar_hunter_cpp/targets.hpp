#ifndef SOLAR_HUNTER_CPP_TARGETS_HPP
#define SOLAR_HUNTER_CPP_TARGETS_HPP

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace solar_hunter {

// A named aim direction for the parabolic reflector.
//
// azimuth: 0 deg = straight ahead (+X), increases counter-clockwise
//          looking from above (toward +Y).
// elevation: 0 deg = horizon, +90 deg = straight up (+Z).
struct Target {
  std::string name;
  double azimuth_deg;
  double elevation_deg;
  std::string note;

  // Unit vector in the reflector base frame (x fwd, y left, z up).
  std::array<double, 3> direction_vector() const;
};

// Registry of named targets. These are fixed placeholder directions, not
// live-tracked positions — swap in a real tracking feed (ephemeris, radar,
// lidar) to replace any of them.
class Targets {
public:
  static std::vector<std::string> list_targets();
  static const Target& get_target(const std::string& name);
  static bool has_target(const std::string& name);
};

}  // namespace solar_hunter

#endif  // SOLAR_HUNTER_CPP_TARGETS_HPP
