#include <solar_hunter_cpp/targets.hpp>
#include <solar_hunter_cpp/reflector_angle.hpp>

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace solar_hunter {

std::array<double, 3> Target::direction_vector() const {
  const double az = deg2rad(azimuth_deg);
  const double el = deg2rad(elevation_deg);
  const double x = std::cos(el) * std::cos(az);
  const double y = std::cos(el) * std::sin(az);
  const double z = std::sin(el);
  std::array<double, 3> v = {x, y, z};
  return normalize_vec3(v);
}

namespace {

struct TargetEntry {
  std::string name;
  double azimuth_deg;
  double elevation_deg;
  std::string note;
};

const TargetEntry kTargetTable[] = {
    {"locusts", 20.0, 5.0,
     "Low-elevation sweep across a field in front of the rig."},
    {"meteors", 0.0, 55.0,
     "High sky elevation, roughly where meteor trails are visible."},
    {"space_debris", -30.0, 70.0,
     "Near-zenith, typical low-earth-orbit pass angle."},
    {"moon_south_darkside", 45.0, 35.0,
     "Placeholder direction for the Moon's southern dark limb; replace with "
     "real ephemeris az/el for the observing site+time."},
    {"solar_sail", 10.0, 80.0,
     "Near-zenith deep-space direction for a solar sail craft."},
};

std::string normalize_target_name(const std::string& name) {
  std::string out = name;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  for (auto& c : out) {
    if (c == ' ' || c == '-') c = '_';
  }

  // The object-detection classifier (mearm_object_detection, labels.txt)
  // emits singular class names ("meteor", "locust") while this target
  // table uses plural names. Map classifier output -> table key here so
  // stage 1 (classify) and stage 3 (aim) agree without renaming either side.
  static const std::map<std::string, std::string> kClassifierAliases = {
      {"meteor", "meteors"},
      {"locust", "locusts"},
  };
  auto alias_it = kClassifierAliases.find(out);
  if (alias_it != kClassifierAliases.end()) {
    out = alias_it->second;
  }
  return out;
}

}  // namespace

std::vector<std::string> Targets::list_targets() {
  std::vector<std::string> names;
  names.reserve(sizeof(kTargetTable) / sizeof(kTargetTable[0]));
  for (const auto& e : kTargetTable) names.push_back(e.name);
  return names;
}

bool Targets::has_target(const std::string& name) {
  const std::string key = normalize_target_name(name);
  for (const auto& e : kTargetTable) {
    if (normalize_target_name(e.name) == key) return true;
  }
  return false;
}

const Target& Targets::get_target(const std::string& name) {
  const std::string key = normalize_target_name(name);
  static Target locusts{"locusts", 20.0, 5.0,
                        "Low-elevation sweep across a field in front of the rig."};
  static Target meteors{"meteors", 0.0, 55.0,
                        "High sky elevation, roughly where meteor trails are visible."};
  static Target space_debris{"space_debris", -30.0, 70.0,
                             "Near-zenith, typical low-earth-orbit pass angle."};
  static Target moon_south_darkside{
      "moon_south_darkside", 45.0, 35.0,
      "Placeholder direction for the Moon's southern dark limb; replace with "
      "real ephemeris az/el for the observing site+time."};
  static Target solar_sail{"solar_sail", 10.0, 80.0,
                           "Near-zenith deep-space direction for a solar sail craft."};

  static const std::map<std::string, const Target*> kMap = [] {
    std::map<std::string, const Target*> m;
    m["locusts"] = &locusts;
    m["meteors"] = &meteors;
    m["space_debris"] = &space_debris;
    m["moon_south_darkside"] = &moon_south_darkside;
    m["solar_sail"] = &solar_sail;
    return m;
  }();

  auto it = kMap.find(key);
  if (it == kMap.end()) {
    std::string valid;
    for (size_t i = 0; i < sizeof(kTargetTable) / sizeof(kTargetTable[0]); ++i) {
      if (i) valid += ", ";
      valid += kTargetTable[i].name;
    }
    throw std::runtime_error("Unknown target '" + name + "'. Valid targets: " + valid);
  }
  return *it->second;
}

}  // namespace solar_hunter
