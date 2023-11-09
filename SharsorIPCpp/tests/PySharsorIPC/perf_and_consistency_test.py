import unittest
import numpy as np

import timeit

from SharsorIPCpp.PySharsorIPC import *

namespace = "PerfTests"

N_ITERATIONS = 1
N_ITERATIONS_STR = 100000

N_ROWS = 100
N_COLS = 60

STR_TENSOR_LENGTH = 100

class PerfThresholds():

    def __init__(self,
            dtype):

        # [us] - microseconds
        if dtype == dtype.Bool:

            self.READ_T_MAX_THRESH = 20
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 20
            self.WRITE_T_AVRG_THRESH = 1000

        if dtype == dtype.Int:

            self.READ_T_MAX_THRESH = 20
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 20
            self.WRITE_T_AVRG_THRESH = 1000

        if dtype == dtype.Float:

            self.READ_T_MAX_THRESH = 20
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 20
            self.WRITE_T_AVRG_THRESH = 1000

        if dtype == dtype.Double:

            self.READ_T_MAX_THRESH = 20
            self.WRITE_T_MAX_THRESH = 1000
            self.READ_T_AVRG_THRESH = 20
            self.WRITE_T_AVRG_THRESH = 1000

class TestPerfBenchBase(unittest.TestCase):

    # we only benchmark the Server write and
    # read method since the Client's versions
    # perform the same operations

    def setUp(self):

        self.rows = N_ROWS
        self.cols = N_COLS
        self.iterations = N_ITERATIONS

        self.is_release = isRelease()

        self.thresholds = PerfThresholds(self.data_type)

        self.server_write = ServerFactory(self.rows,
                                    self.cols,
                                    basename="PySharsor_write" + \
                                    str(self.layout)  +
                                    str(self.data_type),
                                    namespace=namespace,
                                    verbose=True,
                                    vlevel=VLevel.V3,
                                    force_reconnection = True,
                                    dtype=self.data_type,
                                    layout=self.layout)

        self.server_read = ServerFactory(self.rows,
                                    self.cols,
                                    basename="PySharsor_read" + \
                                    str(self.layout)  +
                                    str(self.data_type),
                                    namespace=namespace,
                                    verbose=True,
                                    vlevel=VLevel.V3,
                                    force_reconnection = True,
                                    dtype=self.data_type,
                                    layout=self.layout)
        self.server_write.run()
        self.server_read.run()

        if self.layout == RowMajor:

            self.order = 'C' # 'C'

        if self.layout == ColMajor:

            self.order = 'F' # 'F'

        self.tensor_written = np.zeros((3 * self.server_write.getNRows(), 3 * self.server_write.getNCols()),
                                dtype=toNumpyDType(self.server_write.getScalarType()),
                                order=self.order)
        self.tensor_buffer = np.zeros((3 * self.server_write.getNRows(), 3 * self.server_write.getNCols()),
                                dtype=toNumpyDType(self.server_write.getScalarType()),
                                order=self.order)
        self.tensor_read = np.zeros((3 * self.server_read.getNRows(), 3 * self.server_read.getNCols()),
                                dtype=toNumpyDType(self.server_read.getScalarType()),
                                order=self.order)
        self.read_times = [] # microseconds
        self.write_times = [] # microseconds

        self.consistency_checks = [] # True -> OK; False -> failed

    def tearDown(self):

        self.server_write.close()
        self.server_read.close()

    def PerfAndConsistency(self):

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

            self.randomize(self.tensor_written)

            row = self.server_write.getNRows()
            col = self.server_write.getNCols()

            writeTime = timeit.timeit(lambda: self.server_write.write(self.tensor_written[row:2*row, col:2*col], row, col),
                                                    number=1) # we write the tensor and profile the performance)
            self.write_times.append(writeTime * 1e6)  # Convert to microseconds

            self.server_write.read(self.tensor_buffer[row:2*row, col:2*col], row, col) # we read the tensor

            self.server_read.write(self.tensor_buffer[row:2*row, col:2*col], row, col) # we write it on the other memory

            readTime = timeit.timeit(lambda: self.server_read.read(self.tensor_read[row:2*row, col:2*col], row, col),
                                                    number=1) # and then read it again (and profile the performance)
            self.read_times.append(readTime * 1e6)

            # we check that tensor_written and tensor_read match
            self.consistency_checks.append(self.check_equal(self.tensor_written, self.tensor_read))

        # test post-processing
        Journal.log(self.__class__.__name__,
                    "PerfAndConsistency",
                    "running post-processing steps...\n",
                    LogType.STAT)

        # performance
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
                    "PerfAndConsistency",
                    post_proc_message,
                    LogType.STAT)

        # consistency
        n_of_failures = sum(1 for x in self.consistency_checks if not x)
        consistency_message = f"Number of consistency failures: {n_of_failures}/{self.iterations}\n"
        Journal.log(self.__class__.__name__,
                            "PerfAndConsistency",
                            consistency_message,
                            LogType.STAT)

        # Checking if perf. req. were met
        self.assertLess(averageReadTime, self.thresholds.READ_T_AVRG_THRESH)
        self.assertLess(averageWriteTime, self.thresholds.WRITE_T_AVRG_THRESH)
        # Uncomment below if needed
        # self.assertLess(maxReadTime, self.thresholds.READ_T_MAX_THRESH)
        # self.assertLess(maxWriteTime, self.thresholds.WRITE_T_MAX_THRESH)

        # Checking data consistency
        self.assertTrue(all(self.consistency_checks))

    def check_equal(self,
                A, B):

        if A.dtype == np.float32 or \
            A.dtype == np.float64 or \
            A.dtype == np.float16 or \
            A.dtype == np.float128:

            # For floating point numbers, use machine precision
            return np.allclose(A, B, atol=np.finfo(A.dtype).eps, rtol=0)

        elif A.dtype == np.int64 or A.dtype == np.int32 or A.dtype == np.int16 or A.dtype == np.int8 or \
             A.dtype == np.uint64 or A.dtype == np.uint32 or A.dtype == np.uint16 or A.dtype == np.uint8 or \
             A.dtype == np.bool:

            # For integers and booleans, check exact equality

            return np.array_equal(A, B)

        else:

            raise ValueError("Unsupported data type.")

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

class TestPerfBenchBoolRowMaj(TestPerfBenchBase):

    data_type = dtype.Bool
    layout = RowMajor

    def test_write_read(self):

        self.PerfAndConsistency()

#class TestPerfBenchBoolColMaj(TestPerfBenchBase):

#    data_type = dtype.Bool
#    layout = ColMajor

#    def test_write_read(self):

#        self.PerfAndConsistency()

#class TestPerfBenchIntRowMaj(TestPerfBenchBase):

#    data_type = dtype.Int
#    layout = RowMajor

#    def test_write_read(self):

#        self.PerfAndConsistency()

#class TestPerfBenchFloatRowMaj(TestPerfBenchBase):

#    data_type = dtype.Float
#    layout = RowMajor

#    def test_write_read(self):

#        self.PerfAndConsistency()

#class TestPerfBenchDoubleRowMaj(TestPerfBenchBase):

#    data_type = dtype.Double
#    layout = RowMajor

#    def test_write_read(self):

#        self.PerfAndConsistency()

#class TestPerfBenchIntColMaj(TestPerfBenchBase):

#    data_type = dtype.Int
#    layout = ColMajor

#    def test_write_read(self):

#        self.PerfAndConsistency()

#class TestPerfBenchFloatColMaj(TestPerfBenchBase):

#    data_type = dtype.Float
#    layout = ColMajor

#    def test_write_read(self):

#        self.PerfAndConsistency()

#class TestPerfBenchDoubleColMaj(TestPerfBenchBase):

#    data_type = dtype.Double
#    layout = ColMajor

#    def test_write_read(self):

#        self.PerfAndConsistency()


if __name__ == "__main__":

    unittest.main()
