"""
reflector_angle.py
Core aiming math for the parabolic reflector mounted on the MeArm
gripper.

Physical model:
- The parabolic reflector's focal length is FOCUS_DISTANCE_M (1 m).
- The white-LED / yellow-sunlight blob detected by detector.py is used
  as a point light source placed at (or kept near) that focal point.
- For a parabolic mirror, rays from a point source AT the focus
  reflect off the dish and leave travelling PARALLEL to the mirror's
  own axis, regardless of where on the dish they hit. That means once
  the source is centered on the focus, the outgoing collimated beam
  direction is simply the mirror's axis direction — so "aim the beam
  at a target" reduces to "point the mirror axis at the target's
  direction vector".

This module provides two pieces:
1. mirror_axis_to_servo_angles(): converts a desired 3D aim direction
   into the MeArm gripper's pan/tilt servo angles.
2. focus_alignment_correction(): a small-angle feedback term that
   nudges pan/tilt so the detected blob stays centered in the camera
   frame (i.e. stays sitting on the reflector's focal point) before
   trusting the aim-direction math above.
"""

from dataclasses import dataclass
import numpy as np

from detector import BlobResult

FOCUS_DISTANCE_M = 1.0  # parabola focal length, per the spec

# Approximate camera horizontal/vertical field of view — adjust to the
# actual webcam/lens in use. Used only to convert pixel error into an
# angular error for the focus-centering feedback loop.
CAMERA_HFOV_DEG = 60.0
CAMERA_VFOV_DEG = 45.0

# Proportional gain for the focus-centering correction (dimensionless,
# applied to the raw angular pixel error).
CENTERING_GAIN = 0.5

# MeArm gripper servo limits (degrees) — adjust to the actual rig.
PAN_SERVO_RANGE_DEG = (-90.0, 90.0)
TILT_SERVO_RANGE_DEG = (0.0, 90.0)


@dataclass
class AimResult:
    pan_deg: float
    tilt_deg: float
    azimuth_deg: float
    elevation_deg: float
    clipped: bool


def _clip(value: float, lo: float, hi: float) -> (float, bool):
    clipped = value < lo or value > hi
    return float(np.clip(value, lo, hi)), clipped


def mirror_axis_to_servo_angles(direction_vector: np.ndarray) -> AimResult:
    """Convert a unit aim-direction vector (x fwd, y left, z up) into
    MeArm gripper pan/tilt servo angles that orient the reflector's
    axis along that direction.
    """
    v = np.asarray(direction_vector, dtype=float)
    v = v / np.linalg.norm(v)
    x, y, z = v

    azimuth_deg = np.degrees(np.arctan2(y, x))
    elevation_deg = np.degrees(np.arctan2(z, np.hypot(x, y)))

    # MeArm pan servo rotates the base about Z -> directly tracks azimuth.
    # MeArm tilt servo (wrist) raises the gripper -> directly tracks elevation.
    pan_deg, pan_clipped = _clip(azimuth_deg, *PAN_SERVO_RANGE_DEG)
    tilt_deg, tilt_clipped = _clip(elevation_deg, *TILT_SERVO_RANGE_DEG)

    return AimResult(
        pan_deg=pan_deg,
        tilt_deg=tilt_deg,
        azimuth_deg=azimuth_deg,
        elevation_deg=elevation_deg,
        clipped=pan_clipped or tilt_clipped,
    )


def reflect_incident_to_target(incident_dir: np.ndarray,
                                target_dir: np.ndarray) -> np.ndarray:
    """General law-of-reflection solve: given a unit incident ray
    direction (pointing INTO the mirror) and a desired outgoing unit
    direction (pointing AWAY from the mirror toward the target), return
    the mirror surface normal that makes that reflection happen.

    Used only for the case where the source is NOT exactly at the
    focus (e.g. during the alignment phase) and a direct normal solve
    is needed instead of the focus-simplified axis rule above.

    normal = normalize(target_dir - incident_dir)
    (bisector of the outgoing ray and the reversed incident ray).
    """
    d_in = np.asarray(incident_dir, dtype=float)
    d_in = d_in / np.linalg.norm(d_in)
    d_out = np.asarray(target_dir, dtype=float)
    d_out = d_out / np.linalg.norm(d_out)

    normal = d_out - d_in
    norm_len = np.linalg.norm(normal)
    if norm_len < 1e-9:
        # Incident and target directions coincide -> any normal
        # perpendicular to the ray works; default to "straight up".
        return np.array([0.0, 0.0, 1.0])
    return normal / norm_len


def focus_alignment_correction(blob: BlobResult):
    """Given a detected blob, return (pan_correction_deg, tilt_correction_deg)
    to re-center it in the camera frame, i.e. keep it sitting on the
    reflector's focal point (FOCUS_DISTANCE_M in front of the dish).
    Returns (0.0, 0.0) if no blob was found or it is already centered.
    """
    if not blob.found or blob.frame_width == 0 or blob.frame_height == 0:
        return 0.0, 0.0

    half_w = blob.frame_width / 2.0
    half_h = blob.frame_height / 2.0

    err_x = (blob.cx - half_w) / half_w      # -1..1, + = blob right of center
    err_y = (blob.cy - half_h) / half_h      # -1..1, + = blob below center

    pan_correction_deg = -CENTERING_GAIN * (err_x * CAMERA_HFOV_DEG / 2.0)
    tilt_correction_deg = -CENTERING_GAIN * (err_y * CAMERA_VFOV_DEG / 2.0)

    return pan_correction_deg, tilt_correction_deg


if __name__ == "__main__":
    import targets

    for name in targets.list_targets():
        t = targets.get_target(name)
        aim = mirror_axis_to_servo_angles(t.direction_vector())
        print(f"{name:22s} -> pan={aim.pan_deg:7.2f} deg  "
              f"tilt={aim.tilt_deg:7.2f} deg  (clipped={aim.clipped})")
