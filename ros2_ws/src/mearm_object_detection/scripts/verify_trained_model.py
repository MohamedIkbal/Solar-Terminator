#!/usr/bin/env python3
"""Sanity-checks the trained space_object_classifier.onnx with onnxruntime
only (run as its own process - importing onnx and onnxruntime together in
one process crashes on this system with a duplicate protobuf descriptor).

Loads each original training photo (no augmentation), runs it through the
model with the exact same preprocessing the C++ node uses (resize 64x64,
BGR->RGB, scale to [0,1], NCHW), and reports per-class accuracy.
"""
import os

import cv2
import numpy as np
import onnxruntime as ort

SCRIPT_DIR = os.path.dirname(__file__)
IMAGES_DIR = os.path.join(SCRIPT_DIR, "..", "training_images")
MODEL_PATH = os.path.join(SCRIPT_DIR, "..", "models", "space_object_classifier.onnx")
LABELS_PATH = os.path.join(SCRIPT_DIR, "..", "models", "labels.txt")
IMG = 64


def main():
    with open(LABELS_PATH) as f:
        classes = [line.strip() for line in f if line.strip()]
    print(f"labels.txt classes: {classes}")

    sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])

    total, correct = 0, 0
    per_class = {c: [0, 0] for c in classes}  # name -> [correct, total]
    for ci, cname in enumerate(classes):
        folder_name = cname if cname != "moon_south_darkside" else "moon_dark_side"
        folder = os.path.join(IMAGES_DIR, folder_name)
        if not os.path.isdir(folder):
            continue
        for fn in sorted(os.listdir(folder)):
            if not fn.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                continue
            path = os.path.join(folder, fn)
            bgr = cv2.imread(path, cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            resized = cv2.resize(bgr, (IMG, IMG))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            inp = rgb.transpose(2, 0, 1)[None, ...].astype(np.float32)
            out = sess.run(["output"], {"input": inp})[0][0]
            pred = int(out.argmax())
            ok = int(pred == ci)
            total += 1
            correct += ok
            per_class[cname][0] += ok
            per_class[cname][1] += 1
            print(f"{cname:22s} {fn:16s} -> pred={classes[pred]:22s} "
                  f"conf={out[pred]:.3f} {'OK' if ok else 'WRONG'}")

    print("\nPer-class accuracy:")
    for c, (ok, tot) in per_class.items():
        print(f"  {c:22s} {ok}/{tot}")
    print(f"\nOverall: {correct}/{total} correct ({100.0*correct/max(total,1):.1f}%)")


if __name__ == "__main__":
    main()
