#!/usr/bin/env python3
"""Builds the space-object classifier ONNX graph used by mearm_object_detection.

The network is a small CNN (conv -> relu -> pool) x2 -> global average pool ->
fully-connected -> softmax, built directly with onnx.helper so no training
framework is required at build time. Weights are initialized with a fixed
random seed. This gives a fully working OpenCV-crop -> ONNX-classify pipeline
end to end; swap in weights from a real training run (same input/output
contract: NCHW float32 1x3x64x64 in, 1x5 softmax probabilities out, class
order matching labels.txt) once labeled imagery for the five classes is
available.
"""
import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto

RNG = np.random.default_rng(42)
IMG = 64
CLASSES = [
    "space_debris",
    "meteor",
    "locust",
    "solar_sail",
    "moon_south_darkside",
]


def conv_init(out_c, in_c, k):
    fan_in = in_c * k * k
    scale = np.sqrt(2.0 / fan_in)
    return (RNG.standard_normal((out_c, in_c, k, k)) * scale).astype(np.float32)


def fc_init(out_f, in_f):
    scale = np.sqrt(2.0 / in_f)
    return (RNG.standard_normal((out_f, in_f)) * scale).astype(np.float32)


def build(weights=None):
    initializers = []

    if weights is None:
        w1 = conv_init(8, 3, 3)
        b1 = np.zeros(8, dtype=np.float32)
        w2 = conv_init(16, 8, 3)
        b2 = np.zeros(16, dtype=np.float32)
        fc_w = fc_init(len(CLASSES), 16)
        fc_b = np.zeros(len(CLASSES), dtype=np.float32)
    else:
        w1 = weights["conv1_w"]
        b1 = weights["conv1_b"]
        w2 = weights["conv2_w"]
        b2 = weights["conv2_b"]
        fc_w = weights["fc_w"]
        fc_b = weights["fc_b"]

    for name, arr in [
        ("conv1_w", w1), ("conv1_b", b1),
        ("conv2_w", w2), ("conv2_b", b2),
        ("fc_w", fc_w), ("fc_b", fc_b),
    ]:
        initializers.append(numpy_helper.from_array(arr, name=name))

    nodes = [
        helper.make_node("Conv", ["input", "conv1_w", "conv1_b"], ["conv1"],
                          pads=[1, 1, 1, 1], strides=[1, 1]),
        helper.make_node("Relu", ["conv1"], ["relu1"]),
        helper.make_node("MaxPool", ["relu1"], ["pool1"],
                          kernel_shape=[2, 2], strides=[2, 2]),
        helper.make_node("Conv", ["pool1", "conv2_w", "conv2_b"], ["conv2"],
                          pads=[1, 1, 1, 1], strides=[1, 1]),
        helper.make_node("Relu", ["conv2"], ["relu2"]),
        helper.make_node("MaxPool", ["relu2"], ["pool2"],
                          kernel_shape=[2, 2], strides=[2, 2]),
        helper.make_node("GlobalAveragePool", ["pool2"], ["gap"]),
        helper.make_node("Flatten", ["gap"], ["flat"], axis=1),
        helper.make_node("Gemm", ["flat", "fc_w", "fc_b"], ["logits"],
                          alpha=1.0, beta=1.0, transB=1),
        helper.make_node("Softmax", ["logits"], ["output"], axis=1),
    ]

    graph = helper.make_graph(
        nodes,
        "space_object_classifier",
        [helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, 3, IMG, IMG])],
        [helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, len(CLASSES)])],
        initializers,
    )
    model = helper.make_model(graph, producer_name="mearm_object_detection",
                               opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    return model


if __name__ == "__main__":
    import os
    out_dir = os.path.join(os.path.dirname(__file__), "..", "models")
    os.makedirs(out_dir, exist_ok=True)
    model = build()
    onnx_path = os.path.join(out_dir, "space_object_classifier.onnx")
    onnx.save(model, onnx_path)
    with open(os.path.join(out_dir, "labels.txt"), "w") as f:
        f.write("\n".join(CLASSES) + "\n")
    print(f"wrote {onnx_path}")
    print(f"wrote {os.path.join(out_dir, 'labels.txt')}")
