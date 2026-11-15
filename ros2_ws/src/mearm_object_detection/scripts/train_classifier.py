#!/usr/bin/env python3
"""Trains the space-object classifier on the photos under training_images/
using a from-scratch NumPy forward/backward pass (no torch/tensorflow
available on this system), then bakes the learned weights into the same
ONNX graph shape used by generate_classifier_model.py.

Input contract matches the C++ node exactly: NCHW float32 1x3x64x64, RGB,
pixel values scaled to [0,1].
"""
import os
import sys

import cv2
import numpy as np
import onnx

sys.path.insert(0, os.path.dirname(__file__))
from generate_classifier_model import build, CLASSES, IMG  # noqa: E402

RNG = np.random.default_rng(0)
SCRIPT_DIR = os.path.dirname(__file__)
IMAGES_DIR = os.path.join(SCRIPT_DIR, "..", "training_images")
MODELS_DIR = os.path.join(SCRIPT_DIR, "..", "models")

# ---------------------------------------------------------------------------
# Data loading + augmentation (5 photos/class -> ~400 augmented samples/class)
# ---------------------------------------------------------------------------

def load_raw_images():
    """Returns list of (rgb_float_image_HWC_at_IMGxIMG, class_idx)."""
    samples = []
    for ci, cname in enumerate(CLASSES):
        folder_name = cname if cname != "moon_south_darkside" else "moon_dark_side"
        folder = os.path.join(IMAGES_DIR, folder_name)
        if not os.path.isdir(folder):
            print(f"WARNING: no folder for class '{cname}' at {folder}")
            continue
        files = sorted(
            f for f in os.listdir(folder)
            if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))
        )
        for fn in files:
            path = os.path.join(folder, fn)
            bgr = cv2.imread(path, cv2.IMREAD_COLOR)
            if bgr is None:
                print(f"WARNING: failed to read {path}")
                continue
            resized = cv2.resize(bgr, (IMG, IMG))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            samples.append((rgb, ci))
    return samples


def augment(img):
    """img: HWC float32 [0,1] at IMGxIMG. Returns an augmented copy, same size."""
    out = img.copy()
    # random horizontal flip
    if RNG.random() < 0.5:
        out = out[:, ::-1, :].copy()
    # random rotation +/-20deg with border replicate
    angle = RNG.uniform(-20, 20)
    M = cv2.getRotationMatrix2D((IMG / 2, IMG / 2), angle, 1.0)
    out = cv2.warpAffine(out, M, (IMG, IMG), borderMode=cv2.BORDER_REFLECT101)
    # random brightness/contrast jitter
    gain = RNG.uniform(0.75, 1.25)
    bias = RNG.uniform(-0.08, 0.08)
    out = np.clip(out * gain + bias, 0.0, 1.0)
    # small random crop + resize back (scale jitter)
    pad = RNG.integers(0, 7)
    if pad > 0:
        h, w = out.shape[:2]
        x0 = RNG.integers(0, pad + 1)
        y0 = RNG.integers(0, pad + 1)
        cropped = out[y0:h - (pad - y0) or h, x0:w - (pad - x0) or w]
        out = cv2.resize(cropped, (IMG, IMG))
    # mild gaussian noise
    out = np.clip(out + RNG.normal(0, 0.02, out.shape).astype(np.float32), 0.0, 1.0)
    return out.astype(np.float32)


def build_dataset(raw, per_image=80):
    xs, ys = [], []
    for img, label in raw:
        # include the clean image itself once
        xs.append(img)
        ys.append(label)
        for _ in range(per_image):
            xs.append(augment(img))
            ys.append(label)
    X = np.stack(xs).transpose(0, 3, 1, 2).astype(np.float32)  # N,C,H,W
    Y = np.array(ys, dtype=np.int64)
    return X, Y


# ---------------------------------------------------------------------------
# NumPy CNN: conv(3x3,pad1) -> relu -> maxpool2x2 -> conv(3x3,pad1) -> relu
#            -> maxpool2x2 -> global-avg-pool -> flatten -> fc -> softmax
# ---------------------------------------------------------------------------

def im2col(x, k, pad):
    """x: N,C,H,W -> cols: N, C*k*k, Hout*Wout ; also returns Hout,Wout."""
    n, c, h, w = x.shape
    xp = np.pad(x, ((0, 0), (0, 0), (pad, pad), (pad, pad)))
    hout, wout = h, w  # stride 1, same padding
    cols = np.zeros((n, c, k, k, hout, wout), dtype=x.dtype)
    for i in range(k):
        for j in range(k):
            cols[:, :, i, j, :, :] = xp[:, :, i:i + hout, j:j + wout]
    cols = cols.reshape(n, c * k * k, hout * wout)
    return cols, hout, wout


def conv_forward(x, w, b, pad=1):
    n, c, h, wd = x.shape
    oc, ic, k, _ = w.shape
    cols, hout, wout = im2col(x, k, pad)
    w_flat = w.reshape(oc, ic * k * k)
    out = np.einsum('oc,ncp->nop', w_flat, cols) + b.reshape(1, oc, 1)
    out = out.reshape(n, oc, hout, wout)
    cache = (x, w, b, cols, pad, k)
    return out, cache


def conv_backward(dout, cache):
    x, w, b, cols, pad, k = cache
    n, c, h, wd = x.shape
    oc = w.shape[0]
    hout, wout = dout.shape[2], dout.shape[3]
    dout_flat = dout.reshape(n, oc, hout * wout)
    w_flat = w.reshape(oc, c * k * k)

    db = dout_flat.sum(axis=(0, 2))
    dw_flat = np.einsum('nop,ncp->oc', dout_flat, cols)
    dw = dw_flat.reshape(w.shape)

    dcols = np.einsum('oc,nop->ncp', w_flat, dout_flat)
    dcols = dcols.reshape(n, c, k, k, hout, wout)
    dxp = np.zeros((n, c, h + 2 * pad, wd + 2 * pad), dtype=x.dtype)
    for i in range(k):
        for j in range(k):
            dxp[:, :, i:i + hout, j:j + wout] += dcols[:, :, i, j, :, :]
    dx = dxp[:, :, pad:pad + h, pad:pad + wd]
    return dx, dw, db


def relu_forward(x):
    mask = x > 0
    return x * mask, mask


def relu_backward(dout, mask):
    return dout * mask


def maxpool_forward(x, k=2, s=2):
    n, c, h, w = x.shape
    hout, wout = h // s, w // s
    xr = x[:, :, :hout * s, :wout * s].reshape(n, c, hout, s, wout, s)
    out = xr.max(axis=(3, 5))
    # build argmax mask for backward
    out_b = out.reshape(n, c, hout, 1, wout, 1)
    mask = (xr == out_b).astype(x.dtype)
    # avoid double-counting ties
    counts = mask.sum(axis=(3, 5), keepdims=True)
    mask = mask / np.maximum(counts, 1)
    cache = (mask, x.shape, s)
    return out, cache


def maxpool_backward(dout, cache):
    mask, xshape, s = cache
    n, c, hout, wout = dout.shape
    dout_b = dout.reshape(n, c, hout, 1, wout, 1)
    dxr = mask * dout_b
    dx = np.zeros(xshape, dtype=dout.dtype)
    dx[:, :, :hout * s, :wout * s] = dxr.reshape(n, c, hout * s, wout * s)
    return dx


def gap_forward(x):
    n, c, h, w = x.shape
    out = x.mean(axis=(2, 3))
    cache = (h, w)
    return out, cache


def gap_backward(dout, cache):
    h, w = cache
    n, c = dout.shape
    dx = np.broadcast_to((dout / (h * w)).reshape(n, c, 1, 1), (n, c, h, w)).copy()
    return dx


def fc_forward(x, w, b):
    # w: (out,in), matches Gemm(transB=1): y = x @ w.T + b
    out = x @ w.T + b
    cache = (x, w)
    return out, cache


def fc_backward(dout, cache):
    x, w = cache
    dx = dout @ w
    dw = dout.T @ x
    db = dout.sum(axis=0)
    return dx, dw, db


def softmax_xent_forward_backward(logits, labels):
    z = logits - logits.max(axis=1, keepdims=True)
    exp = np.exp(z)
    probs = exp / exp.sum(axis=1, keepdims=True)
    n = logits.shape[0]
    log_likelihood = -np.log(np.clip(probs[np.arange(n), labels], 1e-9, 1.0))
    loss = log_likelihood.mean()
    dlogits = probs.copy()
    dlogits[np.arange(n), labels] -= 1
    dlogits /= n
    acc = (probs.argmax(axis=1) == labels).mean()
    return loss, acc, dlogits, probs


class Model:
    def __init__(self):
        rng = np.random.default_rng(42)

        def conv_init(oc, ic, k):
            scale = np.sqrt(2.0 / (ic * k * k))
            return (rng.standard_normal((oc, ic, k, k)) * scale).astype(np.float32)

        def fc_init(of, inf):
            scale = np.sqrt(2.0 / inf)
            return (rng.standard_normal((of, inf)) * scale).astype(np.float32)

        self.conv1_w = conv_init(8, 3, 3)
        self.conv1_b = np.zeros(8, dtype=np.float32)
        self.conv2_w = conv_init(16, 8, 3)
        self.conv2_b = np.zeros(16, dtype=np.float32)
        self.fc_w = fc_init(len(CLASSES), 16)
        self.fc_b = np.zeros(len(CLASSES), dtype=np.float32)

        # Adam state
        self.params = ["conv1_w", "conv1_b", "conv2_w", "conv2_b", "fc_w", "fc_b"]
        self.m = {p: np.zeros_like(getattr(self, p)) for p in self.params}
        self.v = {p: np.zeros_like(getattr(self, p)) for p in self.params}
        self.t = 0

    def forward(self, x):
        c1, cache1 = conv_forward(x, self.conv1_w, self.conv1_b)
        r1, rmask1 = relu_forward(c1)
        p1, pcache1 = maxpool_forward(r1)
        c2, cache2 = conv_forward(p1, self.conv2_w, self.conv2_b)
        r2, rmask2 = relu_forward(c2)
        p2, pcache2 = maxpool_forward(r2)
        g, gcache = gap_forward(p2)
        logits, fcache = fc_forward(g, self.fc_w, self.fc_b)
        caches = (cache1, rmask1, pcache1, cache2, rmask2, pcache2, gcache, fcache)
        return logits, caches

    def backward(self, dlogits, caches):
        cache1, rmask1, pcache1, cache2, rmask2, pcache2, gcache, fcache = caches
        dg, dfc_w, dfc_b = fc_backward(dlogits, fcache)
        dp2 = gap_backward(dg, gcache)
        dr2 = maxpool_backward(dp2, pcache2)
        dc2 = relu_backward(dr2, rmask2)
        dp1, dconv2_w, dconv2_b = conv_backward(dc2, cache2)
        dr1 = maxpool_backward(dp1, pcache1)
        dc1 = relu_backward(dr1, rmask1)
        _, dconv1_w, dconv1_b = conv_backward(dc1, cache1)
        grads = {
            "conv1_w": dconv1_w, "conv1_b": dconv1_b,
            "conv2_w": dconv2_w, "conv2_b": dconv2_b,
            "fc_w": dfc_w, "fc_b": dfc_b,
        }
        return grads

    def adam_step(self, grads, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8, weight_decay=1e-4):
        self.t += 1
        for p in self.params:
            g = grads[p] + weight_decay * getattr(self, p)
            self.m[p] = beta1 * self.m[p] + (1 - beta1) * g
            self.v[p] = beta2 * self.v[p] + (1 - beta2) * (g * g)
            mhat = self.m[p] / (1 - beta1 ** self.t)
            vhat = self.v[p] / (1 - beta2 ** self.t)
            update = lr * mhat / (np.sqrt(vhat) + eps)
            setattr(self, p, getattr(self, p) - update.astype(np.float32))

    def weights_dict(self):
        return {p: getattr(self, p).astype(np.float32) for p in self.params}


def main():
    print("Loading photos...")
    raw = load_raw_images()
    if len(raw) < len(CLASSES):
        print("Not enough images found across classes - aborting.")
        return
    counts = {}
    for _, label in raw:
        counts[CLASSES[label]] = counts.get(CLASSES[label], 0) + 1
    print(f"Loaded {len(raw)} source photos: {counts}")

    print("Building augmented training set...")
    X, Y = build_dataset(raw, per_image=80)
    print(f"Training set: {X.shape[0]} samples, shape {X.shape[1:]}")

    # shuffle + 90/10 train/val split
    idx = RNG.permutation(len(X))
    X, Y = X[idx], Y[idx]
    n_val = max(len(CLASSES), int(0.1 * len(X)))
    X_val, Y_val = X[:n_val], Y[:n_val]
    X_tr, Y_tr = X[n_val:], Y[n_val:]

    model = Model()
    batch_size = 16
    epochs = 18
    n = len(X_tr)
    for epoch in range(epochs):
        perm = RNG.permutation(n)
        ep_loss, ep_acc, nb = 0.0, 0.0, 0
        for start in range(0, n, batch_size):
            bi = perm[start:start + batch_size]
            xb, yb = X_tr[bi], Y_tr[bi]
            logits, caches = model.forward(xb)
            loss, acc, dlogits, _ = softmax_xent_forward_backward(logits, yb)
            grads = model.backward(dlogits, caches)
            model.adam_step(grads, lr=2e-3)
            ep_loss += loss
            ep_acc += acc
            nb += 1
        val_logits, _ = model.forward(X_val)
        val_loss, val_acc, _, _ = softmax_xent_forward_backward(val_logits, Y_val)
        print(f"epoch {epoch+1:2d}/{epochs}  train_loss={ep_loss/nb:.4f} "
              f"train_acc={ep_acc/nb:.3f}  val_loss={val_loss:.4f} val_acc={val_acc:.3f}")

    # ---- export to ONNX with the trained weights ----
    weights = model.weights_dict()
    onnx_model = build(weights=weights)
    os.makedirs(MODELS_DIR, exist_ok=True)
    out_path = os.path.join(MODELS_DIR, "space_object_classifier.onnx")
    onnx.save(onnx_model, out_path)
    print(f"wrote trained model to {out_path}")
    print("Run verify_trained_model.py in a separate process to sanity-check "
          "it with onnxruntime (onnx + onnxruntime can't share one process here).")


if __name__ == "__main__":
    main()
