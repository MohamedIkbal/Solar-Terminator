"""
targets.py
Named aim directions for the solar-hunter reflector.

The brief asks the parabolic reflector to redirect the captured light
(white LED / yellow sunlight) toward a set of named targets: locusts,
meteors, space debris, the Moon's south dark-side, and a solar sail.
None of these are things a webcam + MeArm rig can actually track in
real time, so each one is modeled as a fixed, configurable AIM
DIRECTION (azimuth + elevation, degrees, measured from the reflector's
base frame) rather than a tracked position. Swap these numbers for a
real tracking feed (ephemeris, radar, lidar) when one is available —
the rest of the pipeline only needs a unit direction vector.

azimuth: 0 deg = straight ahead (+X), increases counter-clockwise
         looking from above (toward +Y).
elevation: 0 deg = horizon, +90 deg = straight up (+Z).
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class Target:
    name: str
    azimuth_deg: float
    elevation_deg: float
    note: str = ""

    def direction_vector(self) -> np.ndarray:
        """Unit vector in the reflector base frame (x fwd, y left, z up)."""
        az = np.radians(self.azimuth_deg)
        el = np.radians(self.elevation_deg)
        x = np.cos(el) * np.cos(az)
        y = np.cos(el) * np.sin(az)
        z = np.sin(el)
        v = np.array([x, y, z], dtype=float)
        return v / np.linalg.norm(v)


TARGETS = {
    "locusts": Target(
        "locusts", azimuth_deg=20.0, elevation_deg=5.0,
        note="Low-elevation sweep across a field in front of the rig.",
    ),
    "meteors": Target(
        "meteors", azimuth_deg=0.0, elevation_deg=55.0,
        note="High sky elevation, roughly where meteor trails are visible.",
    ),
    "space_debris": Target(
        "space_debris", azimuth_deg=-30.0, elevation_deg=70.0,
        note="Near-zenith, typical low-earth-orbit pass angle.",
    ),
    "moon_south_darkside": Target(
        "moon_south_darkside", azimuth_deg=45.0, elevation_deg=35.0,
        note="Placeholder direction for the Moon's southern dark limb; "
             "replace with real ephemeris az/el for the observing site+time.",
    ),
    "solar_sail": Target(
        "solar_sail", azimuth_deg=10.0, elevation_deg=80.0,
        note="Near-zenith deep-space direction for a solar sail craft.",
    ),
}


# The object-detection classifier (mearm_object_detection, labels.txt) uses
# singular class names ("meteor", "locust") while the target table above
# uses plural names. Map classifier output -> target table key here so the
# two stages (classify -> aim) agree without renaming either side.
CLASSIFIER_LABEL_ALIASES = {
    "meteor": "meteors",
    "locust": "locusts",
    "space_debris": "space_debris",
    "solar_sail": "solar_sail",
    "moon_south_darkside": "moon_south_darkside",
}


def get_target(name: str) -> Target:
    key = name.strip().lower().replace(" ", "_").replace("-", "_")
    key = CLASSIFIER_LABEL_ALIASES.get(key, key)
    if key not in TARGETS:
        valid = ", ".join(TARGETS.keys())
        raise KeyError(f"Unknown target '{name}'. Valid targets: {valid}")
    return TARGETS[key]


def list_targets():
    return list(TARGETS.keys())


if __name__ == "__main__":
    for t in TARGETS.values():
        v = t.direction_vector()
        print(f"{t.name:22s} az={t.azimuth_deg:6.1f} el={t.elevation_deg:6.1f} "
              f"dir={v.round(3)}  -- {t.note}")
