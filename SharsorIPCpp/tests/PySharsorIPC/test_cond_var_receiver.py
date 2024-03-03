import unittest
import numpy as np

import timeit

from SharsorIPCpp.PySharsorIPC import *
from SharsorIPCpp.PySharsorIPC import dtype as sharsor_dtype
from SharsorIPCpp.PySharsorIPC import ConditionVariable
from SharsorIPCpp.PySharsorIPC import toNumpyDType

namespace = "pippo"

client = ClientFactory(
                basename="VarToBeChecked",
                namespace=namespace,
                verbose=True,
                vlevel=VLevel.V2,
                dtype=sharsor_dtype.Int)

client.attach()

n_rows = client.getNRows()
n_cols = client.getNCols()

numpy_view = np.full((n_rows, n_cols),
                0,
                dtype=toNumpyDType(client.getScalarType()))

cond_var1 = ConditionVariable(False, 
                        "CondVarRead", 
                        namespace, 
                        True, 
                        VLevel.V2)

cond_var2 = ConditionVariable(False, 
                        "ConVarWrite", 
                        namespace, 
                        True,
                        VLevel.V2)

client.read(numpy_view[0:1, 0:1], 
                    0, 0)

lock1 = cond_var1.lock()

print("AAAAAAAAA")
cond_var1.wait(lock1.get())

cond_var1.close()
cond_var2.close()