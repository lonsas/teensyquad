import ctypes


quad = ctypes.CDLL("../build/model/teensyquad.so")

py_receiver = [1000] * 6;
receiver = (ctypes.c_int * len(py_receiver))(*py_receiver)

quad.stateInit()
quad.receiverSetAllManualPW(receiver)
for _ in range(10):
    print("State: {0}".format(quad.getCurrState()))
    quad.stateUpdate()

quad.receiverSetManualPW(4, 2000)
for _ in range(10):
    print("State: {0}".format(quad.getCurrState()))
    quad.stateUpdate()
