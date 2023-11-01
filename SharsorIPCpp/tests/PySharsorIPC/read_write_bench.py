import unittest
import numpy as np

import time

from SharsorIPCpp.PySharsorIPC import *

namespace = "PerfTests"

N_ITERATIONS = 1000000;
N_ITERATIONS_STR = 100000;

N_ROWS = 100
N_COLS = 60

STR_TENSOR_LENGTH = 100;

class TestPerfBenchBase(unittest.TestCase):

    def setUp(self):

        self.rows = N_ROWS
        self.cols = N_COLS
        self.iterations = N_ITERATIONS

        self.is_release = isRelease()

        self.server = ServerFactory(self.rows,
                                    self.cols,
                                    basename="PySharsor" + \
                                    str(self.layout)  +
                                    str(self.data_type),
                                    namespace=namespace,
                                    verbose=True,
                                    vlevel=VLevel.V3,
                                    dtype=self.data_type,
                                    layout=self.layout)
        self.server.run()

        if self.layout == RowMajor:

            self.order = 'C'

        if self.layout == ColMajor:

            self.order = 'F'

        self.tensor_copy = np.zeros((self.server.getNRows(), self.server.getNCols()),
                                dtype=toNumpyDType(self.server.getScalarType()), # dtype has to match
                                order=self.order)

    def tearDown(self):

        self.server.close()

    def readAndWrite(self):

        a = 1

class TestPerfBenchBoolColMaj(TestPerfBenchBase):

    data_type = dtype.Bool
    layout = ColMajor

    def test_write_read(self):

        if self.is_release:

            message = "SharsorIPCpp was compiled in Release mode. " + \
                "This is good and will ensure meaningful benchmarking results.\n"

            Journal.log(self.__class__.__name__,
                        "test_write_read",
                        message,
                        LogType.STAT)

        if not self.is_release:

            message = "SharsorIPCpp was not compiled in Release mode. " + \
                "Please compile it in Release mode to get meaningful results.\n"

            Journal.log(self.__class__.__name__,
                        "test_write_read",
                        message,
                        LogType.WARN)

        self.readAndWrite()

if __name__ == "__main__":

    unittest.main()
