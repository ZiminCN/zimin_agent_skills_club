import onnxruntime as onnxrt
import os

sess = onnxrt.InferenceSession("../onnx_model/circle_model.onnx")

input = sess.get_inputs()[0]
output = sess.get_outputs()[0]

print(f"input.name: {input.name}")
print(f"input.shape: {input.shape}")
print(f"input.type: {input.type}")

print(f"output.name: {output.name}")
print(f"output.shape: {output.shape}")
print(f"output.type: {output.type}")
