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

        self.layout = None
        self.dtype = None

        self.server = ServerFactory(self.rows,
                                    self.cols,
                                    basename="PySharsor",
                                    namespace=namespace,
                                    verbose=True,
                                    vlevel=VLevel.V3,
                                    dtype=self.data_type,
                                    layout=self.layout)
        self.server.run()

        if self.layout == RowMajor:

            self.order = 'C'

        if self.layout == RowMajor:

            self.order = 'F'

        self.tensor_copy = np.zeros((server.getNRows(), server.getNCols()),
                                dtype=toNumpyDType(server.getScalarType()), # dtype has to match
                                order=self.order)

    def tearDown(self):

        self.server.close()

    def readAndWrite(self):

        check_comp_type(journal);

        double READ_T_MAX_THRESH = Thresholds<ScalarType, layout>::READ_T_MAX_THRESH;
        double WRITE_T_MAX_THRESH = Thresholds<ScalarType, layout>::WRITE_T_MAX_THRESH;
        double READ_T_AVRG_THRESH = Thresholds<ScalarType, layout>::READ_T_AVRG_THRESH;
        double WRITE_T_AVRG_THRESH = Thresholds<ScalarType, layout>::WRITE_T_AVRG_THRESH;

        std::vector<double> readTimes;
        std::vector<double> readTimesView;
        std::vector<double> writeTimes;

        journal.log("PerfTest", "\nBenchmarking performance...\n",
                    Journal::LogType::STAT);

        Tensor<ScalarType, layout> tensor_other = Tensor<ScalarType, layout>::Zero(3 * this->rows,
                                           2 * this->cols); // tensor of which a view is created

        for (int i = 0; i < this->iterations; ++i) {

            Tensor<ScalarType, layout> myData(this->rows, this->cols);
            myData.setRandom(); // we generate a random tensor of the right size

            // we measure the time to write it on the memory
            auto startWrite = std::chrono::high_resolution_clock::now();
            this->server_ptr->writeTensor(myData);
            auto endWrite = std::chrono::high_resolution_clock::now();
            double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
            writeTimes.push_back(writeTime);

            // we measure the time to read a copy of the tensor
            auto startRead = std::chrono::high_resolution_clock::now();
            this->server_ptr->readTensor(this->tensor_copy, 0, 0);
            auto endRead = std::chrono::high_resolution_clock::now();
            double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
            readTimes.push_back(readTime);

        }

        journal.log("PerfTest", "\nrunning post-processing steps...\n",
                    Journal::LogType::STAT);

        // some post-processing
        double averageReadTime = 0;
        double averageWriteTime = 0;
        double maxReadTime = std::numeric_limits<double>::min();
        double maxWriteTime = std::numeric_limits<double>::min();

        for (int i = 0; i < this->iterations; ++i) {
            averageReadTime += readTimes[i];
            averageWriteTime += writeTimes[i];

            if (readTimes[i] > maxReadTime) {
                maxReadTime = readTimes[i];
            }

            if (writeTimes[i] > maxWriteTime) {
                maxWriteTime = writeTimes[i];
            }
        }

        averageReadTime /= this->iterations;
        averageWriteTime /= this->iterations;

        std::cout << "Number of performed iterations: " << this->iterations << std::endl;
        std::cout << "Average Read (with copy) Time: " << averageReadTime << " ns" << std::endl;
        std::cout << "Average Write Time: " << averageWriteTime << " ns" << std::endl;
        std::cout << "Maximum Read (with copy) Time: " << maxReadTime << " ns" << std::endl;
        std::cout << "Maximum Write Time: " << maxWriteTime << " ns\n" << std::endl;

        // Checking if perf. req. were met

        // reading (avrg)
        ASSERT_LT(averageReadTime, READ_T_AVRG_THRESH);
        ASSERT_LT(averageWriteTime, WRITE_T_AVRG_THRESH);


if __name__ == "__main__":

    unittest.main()
