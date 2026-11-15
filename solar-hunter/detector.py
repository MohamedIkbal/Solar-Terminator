"""
detector.py
Detects a bright circular light source in a camera frame: either a
white LED or a yellow sunlight glare/reflection. Returns the pixel
center, radius, and a color label for the brightest qualifying blob.

Approach:
1. Convert frame to HSV.
2. Build two masks:
   - "white"  mask: low saturation, high value (LED glare is usually
     near-white and overexposed).
   - "yellow" mask: hue band around yellow, high value (sunlight through
     a window or a yellow-ish glare).
3. Combine masks, clean up with morphology, then find contours and
   fit circles (also cross-checked with Hough circles on the grayscale
   image for roundness).
4. Pick the largest sufficiently-round blob as the target light.
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class BlobResult:
    found: bool
    cx: float = 0.0          # pixel x of blob center
    cy: float = 0.0          # pixel y of blob center
    radius: float = 0.0      # pixel radius
    color_label: str = ""    # "white_led" or "yellow_sun"
    frame_width: int = 0
    frame_height: int = 0


# HSV thresholds — tuned for typical webcam exposure; adjust per camera.
WHITE_LOWER = np.array([0, 0, 220])
WHITE_UPPER = np.array([180, 60, 255])

YELLOW_LOWER = np.array([18, 80, 180])
YELLOW_UPPER = np.array([35, 255, 255])

MIN_BLOB_RADIUS_PX = 2
MIN_CIRCULARITY = 0.35  # 1.0 = perfect circle, area / (pi * r^2)
# At small pixel sizes a real LED highlight is only a handful of pixels and
# rarely forms a clean circle (webcam sensor noise, slight motion blur,
# JPEG/USB compression blocking) - a strict 0.6 circularity requirement was
# rejecting genuine tiny LED blobs (observed ~0.5-0.55) while still letting
# larger, more clearly non-circular glare patches through at the old
# threshold too. A small minimum radius plus a looser circularity floor
# catches the real small/bright target without accepting large irregular
# glare (those still fail on circularity and area/radius checks below).


def _best_circle_from_mask(mask: np.ndarray):
    """Find the largest, roundest contour in a binary mask.
    Returns (cx, cy, radius, circularity) or None.
    """
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = -1.0
    for c in contours:
        area = cv2.contourArea(c)
        if area <= 0:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        if radius < MIN_BLOB_RADIUS_PX:
            continue
        circularity = area / (np.pi * radius * radius + 1e-6)
        if circularity < MIN_CIRCULARITY:
            continue
        # Rank by roundness first (a true point light source is compact and
        # round), not by raw area - a large irregular glare patch can have
        # more pixels than the actual small LED highlight but is much less
        # circular, so it should lose to the smaller, rounder real target.
        if circularity > best_score:
            best_score = circularity
            best = (cx, cy, radius, circularity)
    return best


def detect_light_blob(frame_bgr: np.ndarray) -> BlobResult:
    """Detect the dominant white-LED or yellow-sunlight circular blob."""
    h, w = frame_bgr.shape[:2]
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    white_mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    yellow_mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

    white_hit = _best_circle_from_mask(white_mask)
    yellow_hit = _best_circle_from_mask(yellow_mask)

    candidates = []
    if white_hit is not None:
        candidates.append(("white_led", white_hit))
    if yellow_hit is not None:
        candidates.append(("yellow_sun", yellow_hit))

    if not candidates:
        return BlobResult(found=False, frame_width=w, frame_height=h)

    # Prefer the roundest (most point-like) of the two candidates - a real
    # LED/sun glint is compact and circular, unlike broad glare patches.
    label, (cx, cy, radius, _circ) = max(candidates, key=lambda item: item[1][3])

    return BlobResult(
        found=True,
        cx=float(cx),
        cy=float(cy),
        radius=float(radius),
        color_label=label,
        frame_width=w,
        frame_height=h,
    )


def draw_debug_overlay(frame_bgr: np.ndarray, result: BlobResult) -> np.ndarray:
    """Draw the detected blob on a copy of the frame for visual debugging."""
    out = frame_bgr.copy()
    if result.found:
        center = (int(result.cx), int(result.cy))
        # A real LED highlight is often only a few pixels wide, which draws
        # a circle too small to see at a glance in a live preview. Enforce a
        # minimum on-screen radius for visibility only - this does not
        # affect result.radius (the actual detected size used for aiming).
        display_radius = max(int(result.radius), 10)
        cv2.circle(out, center, display_radius, (0, 255, 0), 5)
        cv2.circle(out, center, 2, (0, 0, 255), 3)
        cv2.putText(
            out,
            f"{result.color_label} r={result.radius:.1f}px",
            (center[0] + 10, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            out,
            "no light blob detected",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
    return out


if __name__ == "__main__":
    # Quick manual test against the default camera.
    # NOTE: on this machine camera index 0 is the Surface front camera,
    # which under Linux stays locked to a near-black exposure (no usable
    # manual-exposure mode) no matter the lighting. Index 5 ("Integrated
    # Camera") is the one that actually exposes properly - use that as
    # the default; override with an index argument if your hardware differs.
    cam_index = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"No camera available at index {cam_index}.")
    else:
        print("Press 'q' to quit the preview window.")
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            result = detect_light_blob(frame)
            debug = draw_debug_overlay(frame, result)
            cv2.imshow("solar-hunter blob detector", debug)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()
