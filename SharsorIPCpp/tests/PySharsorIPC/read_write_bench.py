import unittest
import numpy as np

import time

#from SharsorIPCpp.PySharsorIPC import ServerFactory
from SharsorIPCpp.PySharsorIPC import *
#from SharsorIPCpp.PySharsorIPC import VLevel
#from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
#from SharsorIPCpp.PySharsorIPC import dtype, toNumpyDType

namespace = "PerfTests"

N_ITERATIONS = 1000000;
N_ITERATIONS_STR = 100000;

N_ROWS = 100
N_COLS = 60

STR_TENSOR_LENGTH = 100;

class TestAddFunction(unittest.TestCase):

    def test_read_write_bool_row_maj(self):

        # Create a server
        server = ServerFactory(N_ROWS,
                               N_COLS,
                               basename="PySharsorBoolRowMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Bool,
                               layout=RowMajor)

    def read_write_bool_col_maj(self):

        # Create a server
        server = ServerFactory(basename="PySharsorBoolColMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Bool,
                               layout=ColMajor)

    def read_write_int_row_maj(self):

        # Create a server
        server = ServerFactory(basename="PySharsorIntlRowMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Int,
                               layout=RowMajor)

    def read_write_int_col_maj(self):

        # Create a server
        server = ServerFactory(basename="PySharsorIntColMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Int,
                               layout=ColMajor)

    def read_write_float_row_maj(self):

        # Create a server
        server = ServerFactory(basename="PySharsorFloatRowMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Float,
                               layout=RowMajor)

    def read_write_float_col_maj(self):

        # Create a server
        server = ServerFactory(basename="PySharsorFloatColMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Float,
                               layout=ColMajor)

    def read_write_double_row_maj(self):

        # Create a server
        server = ServerFactory(basename="PySharsorDoubleRowMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Double,
                               layout=RowMajor)

    def read_write_double_col_maj(self):

        # Create a server
        server = ServerFactory(basename="PySharsorDoubleColMaj",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Double,
                               layout=ColMajor)

if __name__ == "__main__":

    unittest.main()
