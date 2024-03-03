import unittest
import numpy as np

import timeit

from SharsorIPCpp.PySharsorIPC import *
from SharsorIPCpp.PySharsorIPC import dtype as sharsor_dtype
from SharsorIPCpp.PySharsorIPC import ConditionVariable
from SharsorIPCpp.PySharsorIPC import toNumpyDType

namespace = "pippo"

server = ServerFactory(1,
                1,
                basename="VarToBeChecked",
                namespace=namespace,
                verbose=True,
                vlevel=VLevel.V2,
                force_reconnection = True,
                dtype=sharsor_dtype.Int)

server.run()

numpy_view = np.full((1, 1),
                0,
                dtype=toNumpyDType(server.getScalarType()))

cond_var1 = ConditionVariable(True, 
                        "CondVarRead", 
                        namespace, 
                        True, 
                        VLevel.V2)

cond_var2 = ConditionVariable(True, 
                        "ConVarWrite", 
                        namespace, 
                        True, 
                        VLevel.V2)


server.write(numpy_view[0:1, 0:1], 
                    0, 0)