import sys, numpy as np, onnx
from onnx import helper, TensorProto, numpy_helper
out = sys.argv[1]

def gemm(name, n_in, n_out, W, b):
    X = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, n_in])
    Y = helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, n_out])
    w = numpy_helper.from_array(np.asarray(W, dtype=np.float32), "W")
    bb = numpy_helper.from_array(np.asarray(b, dtype=np.float32), "B")
    node = helper.make_node("Gemm", ["obs", "W", "B"], ["action"], transB=1)
    g = helper.make_graph([node], name, [X], [Y], [w, bb])
    m = helper.make_model(g, opset_imports=[helper.make_opsetid("", 18)])
    m.ir_version = 8
    onnx.checker.check_model(m)
    onnx.save(m, f"{out}/{name}.onnx")

gemm("zero45x12", 45, 12, np.zeros((12, 45)), np.zeros(12))
W = np.zeros((12, 6))
for i in range(12):
    W[i, i % 6] = 0.1 * (i + 1)
gemm("lin6x12", 6, 12, W, np.zeros(12))
gemm("nan6x12", 6, 12, np.zeros((12, 6)), np.full(12, np.nan))
