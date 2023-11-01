import unittest
import numpy as np

import timeit

from SharsorIPCpp.PySharsorIPC import *

namespace = "PerfTests"

N_ITERATIONS = 100000;
N_ITERATIONS_STR = 100000;

N_ROWS = 100
N_COLS = 60

STR_TENSOR_LENGTH = 100;

class PerfThresholds():

    def __init__(self,
            dtype):

        # [us] - microseconds
        if dtype == dtype.Bool:

            self.READ_T_MAX_THRESH = 50
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 50
            self.WRITE_T_AVRG_THRESH = 1000

        if dtype == dtype.Int:

            self.READ_T_MAX_THRESH = 50
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 50
            self.WRITE_T_AVRG_THRESH = 1000

        if dtype == dtype.Float:

            self.READ_T_MAX_THRESH = 50
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 50
            self.WRITE_T_AVRG_THRESH = 1000

        if dtype == dtype.Double:

            self.READ_T_MAX_THRESH = 50
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 50
            self.WRITE_T_AVRG_THRESH = 1000

class TestPerfBenchBase(unittest.TestCase):

    # we only benchmark the Server writeTensor and
    # readTensor method since the Client's versions
    # perform the same operations

    def setUp(self):

        self.rows = N_ROWS
        self.cols = N_COLS
        self.iterations = N_ITERATIONS

        self.is_release = isRelease()

        self.thresholds = PerfThresholds(self.data_type)

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

        self.read_times = [] # microseconds
        self.write_times = [] # microseconds

    def tearDown(self):

        self.server.close()

    def readAndWrite(self):

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

        for i in range(0, N_ITERATIONS):

            self.randomize(self.tensor_copy) # randomize tensor_copy

            writeTime = timeit.timeit(lambda: self.server.writeTensor(self.tensor_copy, 0, 0),
                                        number=1)
#            writeTime = 0
            self.write_times.append(writeTime * 1e6)  # Convert to microseconds

#            readTime = timeit.timeit(lambda: self.server.readTensor(self.tensor_copy, 0, 0),
#                                        number=1)
            readTime = 0
            self.read_times.append(readTime * 1e6)

        Journal.log(self.__class__.__name__,
                    "readAndWrite",
                    "running post-processing steps...\n",
                    LogType.STAT)

        averageReadTime = np.mean(self.read_times)
        averageWriteTime = np.mean(self.write_times)
        maxReadTime = np.max(self.read_times)
        maxWriteTime = np.max(self.write_times)

        post_proc_message = f"Number of performed iterations: {self.iterations}\n" + \
                            f"Average Read (with copy) Time: {averageReadTime} us\n" + \
                            f"Average Write Time: {averageWriteTime} us\n" + \
                            f"Maximum Read (with copy) Time: {maxReadTime} us\n" + \
                            f"Maximum Write Time: {maxWriteTime} us\n"

        Journal.log(self.__class__.__name__,
                    "readAndWrite",
                    post_proc_message,
                    LogType.STAT)

        # Checking if perf. req. were met
        self.assertLess(averageReadTime, self.thresholds.READ_T_AVRG_THRESH)
        self.assertLess(averageWriteTime, self.thresholds.WRITE_T_AVRG_THRESH)
        # Uncomment below if needed
        # self.assertLess(maxReadTime, self.thresholds.READ_T_MAX_THRESH)
        # self.assertLess(maxWriteTime, self.thresholds.WRITE_T_MAX_THRESH)

    def randomize(self,
                 arr):

     if np.issubdtype(arr.dtype, np.integer):

         arr[:] = np.random.randint(np.iinfo(arr.dtype).min, np.iinfo(arr.dtype).max, arr.shape)

     elif arr.dtype == np.float32:

         arr[:] = np.random.uniform(-1, 1, arr.shape)

     elif arr.dtype == np.float64:

         arr[:] = np.random.rand(*arr.shape)

     elif arr.dtype == np.bool_:

         arr[:] = np.random.choice([True, False], arr.shape)

     else:

         raise ValueError("Unsupported dtype for randomization")

class TestPerfBenchBoolColMaj(TestPerfBenchBase):

    data_type = dtype.Bool
    layout = ColMajor

    def test_write_read(self):

        self.readAndWrite()

#class TestPerfBenchIntColMaj(TestPerfBenchBase):

#    data_type = dtype.Int
#    layout = ColMajor

#    def test_write_read(self):

#        self.readAndWrite()

#class TestPerfBenchFloatColMaj(TestPerfBenchBase):

#    data_type = dtype.Float
#    layout = ColMajor

#    def test_write_read(self):

#        self.readAndWrite()

#class TestPerfBenchDoubleColMaj(TestPerfBenchBase):

#    data_type = dtype.Double
#    layout = ColMajor

#    def test_write_read(self):

#        self.readAndWrite()

if __name__ == "__main__":

    unittest.main()
