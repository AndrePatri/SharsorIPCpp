#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <csignal>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/StringTensor.hpp>
#include <SharsorIPCpp/Helpers.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <test_utils.hpp>

int N_ITERATIONS = 1000000;
int N_ITERATIONS_STR = 100000;

int STR_TENSOR_LENGTH = 100;

using namespace SharsorIPCpp;

using VLevel = Journal::VLevel;

static std::string name_space = "PerfTests";

static Journal journal("PerfTests");

template <typename T>
class PerfTest : public ::testing::Test {
protected:
    PerfTest() : rows(100),
                   cols(60),
                   iterations(N_ITERATIONS),
                   server_ptr(new Server<T>(rows, cols,
                                     "SharsorIPCpp",
                                     name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   tensor_copy(rows, cols) {
        server_ptr->run();
    }

    void SetUp() override {
        // Initialization code (if needed)
    }

    void TearDown() override {
        server_ptr->close();
        // Cleanup code (if needed)
    }

    int rows;
    int cols;
    int iterations;
    typename Server<T>::UniquePtr server_ptr;
    Tensor<T> tensor_copy;

};

// Type aliases for simplicity
using PerfTestBool = PerfTest<bool>;
using PerfTestInt = PerfTest<int>;
using PerfTestFloat = PerfTest<float>;
using PerfTestDoubleloat = PerfTest<double>;

// Actual test function
TYPED_TEST_CASE_P(PerfTest);

TYPED_TEST_P(PerfTest, WriteReadBenchmark) {
    // Your test code here. You can use 'TypeParam' as the type.
    // This will be replaced by each of the types you listed below.

    check_comp_type(journal);

    double READ_T_MAX_THRESH = Thresholds<TypeParam>::READ_T_MAX_THRESH;
    double READ_TV_MAX_THRESH = Thresholds<TypeParam>::READ_TV_MAX_THRESH;
    double WRITE_T_MAX_THRESH = Thresholds<TypeParam>::WRITE_T_MAX_THRESH;
    double READ_T_AVRG_THRESH = Thresholds<TypeParam>::READ_T_AVRG_THRESH;
    double READ_TV_AVRG_THRESH = Thresholds<TypeParam>::READ_TV_AVRG_THRESH;
    double WRITE_T_AVRG_THRESH = Thresholds<TypeParam>::WRITE_T_AVRG_THRESH;

    std::vector<double> readTimes;
    std::vector<double> readTimesView;
    std::vector<double> writeTimes;

    journal.log("PerfTest", "\nBenchmarking performance...\n",
                Journal::LogType::STAT);

    Tensor<TypeParam> tensor_other = Tensor<TypeParam>::Zero(3 * this->rows,
                                       2 * this->cols); // tensor of which a view is created

    TensorView<TypeParam> block_view = helpers::createViewFrom<TypeParam>(
                                                    tensor_other,
                                                    this->rows, this->cols, // indeces
                                                    this->rows, this->cols); // dimensions
    // we create a view of the central block of the tensor

    for (int i = 0; i < this->iterations; ++i) {

        Tensor<TypeParam> myData(this->rows, this->cols);
        myData.setRandom(); // we generate a random tensor of the right size

        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        this->server_ptr->writeTensor(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read a copy of the tensor
        auto startRead = std::chrono::high_resolution_clock::now();
        this->server_ptr->readTensor(this->tensor_copy);
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);

        // we measure the time to read a copy of the tensor using a TensorView
        auto startReadView = std::chrono::high_resolution_clock::now();
        this->server_ptr->readTensor(block_view,
                               0, 0);
        auto endReadView = std::chrono::high_resolution_clock::now();
        double readTimeView = std::chrono::duration_cast<std::chrono::nanoseconds>(endReadView - startReadView).count();
        readTimesView.push_back(readTimeView);

    }

    journal.log("PerfTest", "\nrunning post-processing steps...\n",
                Journal::LogType::STAT);

    // some post-processing
    double averageReadTime = 0;
    double averageReadTimeView = 0;
    double averageWriteTime = 0;
    double maxReadTime = std::numeric_limits<double>::min();
    double maxReadTimeView = std::numeric_limits<double>::min();
    double maxWriteTime = std::numeric_limits<double>::min();

    for (int i = 0; i < this->iterations; ++i) {
        averageReadTime += readTimes[i];
        averageReadTimeView += readTimesView[i];
        averageWriteTime += writeTimes[i];

        if (readTimes[i] > maxReadTime) {
            maxReadTime = readTimes[i];
        }

        if (writeTimes[i] > maxWriteTime) {
            maxWriteTime = writeTimes[i];
        }

        if (readTimesView[i] > maxReadTimeView) {
            maxReadTimeView = readTimesView[i];
        }
    }

    averageReadTime /= this->iterations;
    averageReadTimeView /= this->iterations;
    averageWriteTime /= this->iterations;

    std::cout << "Number of performed iterations: " << this->iterations << std::endl;
    std::cout << "Average Read (with copy) Time: " << averageReadTime << " ns" << std::endl;
    std::cout << "Average Read (with copy, into TensorView) Time: " << averageReadTimeView << " ns" << std::endl;
    std::cout << "Average Write Time: " << averageWriteTime << " ns" << std::endl;
    std::cout << "Maximum Read (with copy) Time: " << maxReadTime << " ns" << std::endl;
    std::cout << "Maximum Read (with copy, into TensorView) Time: " << maxReadTimeView << " ns" << std::endl;
    std::cout << "Maximum Write Time: " << maxWriteTime << " ns\n" << std::endl;

    // Checking if perf. req. were met

    // reading (avrg)
    ASSERT_LT(averageReadTime, READ_T_AVRG_THRESH);
    ASSERT_LT(averageReadTimeView, READ_TV_AVRG_THRESH);
    ASSERT_LT(averageWriteTime, WRITE_T_AVRG_THRESH);

    // reading (max)
//    ASSERT_LT(maxReadTime, READ_T_MAX_THRESH);
//    ASSERT_LT(maxReadTimeView, READ_TV_MAX_THRESH);
//    ASSERT_LT(maxWriteTime, WRITE_T_MAX_THRESH);

}

// List of types we want to test
REGISTER_TYPED_TEST_CASE_P(PerfTest, WriteReadBenchmark);
using MyTypes = ::testing::Types<bool, int, float, double>;
INSTANTIATE_TYPED_TEST_CASE_P(MyInstantiation, PerfTest, MyTypes);

class StringTensorWrite : public ::testing::Test {
protected:

    StringTensorWrite() :
                   string_t_ptr(new StringTensor<StrServer>(
                                     STR_TENSOR_LENGTH,
                                     "SharedStrTensor", name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   str_vec_write(STR_TENSOR_LENGTH),
                   str_vec_read(STR_TENSOR_LENGTH){

        for (int i = 0; i < str_vec_write.size(); ++i) {

            str_vec_write[i] = random_string(25); // random initialization
        }

        string_t_ptr->run();

    }

    void SetUp() override {

    }

    void TearDown() override {

        string_t_ptr->close();

    }

    StringTensor<StrServer>::UniquePtr string_t_ptr;

    std::vector<std::string> str_vec_write;

    std::vector<std::string> str_vec_read;

};

TEST_F(StringTensorWrite, StringTensorWriteBenchmark) {

    check_comp_type(journal);

    double READ_T_MAX_THRESH =  10000000; // [nanoseconds], maximum allowed read time
    double WRITE_T_MAX_THRESH = 10000000; // [nanoseconds], maximum allowed read time
    double READ_T_AVRG_THRESH =   15000; // [nanoseconds]
    double WRITE_T_AVRG_THRESH =  15000; // [nanoseconds]

    std::vector<double> readTimes;
    std::vector<double> writeTimes;

    journal.log("ServerTestStringTensor", "\nBenchmarking performance with StringTensor...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < N_ITERATIONS_STR; ++i) {


        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        string_t_ptr->write(str_vec_write, 0); // writes the whole vector
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read it all
        auto startRead = std::chrono::high_resolution_clock::now();
        string_t_ptr->read(str_vec_read, 0);
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);
    }

    journal.log("ServerTestStringTensor", "\nrunning post-processing steps...\n",
                Journal::LogType::STAT);

    // some post-processing
    double averageReadTime = 0;
    double averageWriteTime = 0;
    double maxReadTime = std::numeric_limits<double>::min();
    double maxWriteTime = std::numeric_limits<double>::min();

    for (int i = 0; i < N_ITERATIONS_STR; ++i) {
        averageReadTime += readTimes[i];
        averageWriteTime += writeTimes[i];

        if (readTimes[i] > maxReadTime) {
            maxReadTime = readTimes[i];
        }

        if (writeTimes[i] > maxWriteTime) {
            maxWriteTime = writeTimes[i];
        }
    }

    averageReadTime /= N_ITERATIONS_STR;
    averageWriteTime /= N_ITERATIONS_STR;

    std::cout << "Number of performed iterations: " << N_ITERATIONS_STR << std::endl;
    std::cout << "Average Read (with copy) Time: " << averageReadTime << " ns" << std::endl;
    std::cout << "Average Write Time: " << averageWriteTime << " ns" << std::endl;
    std::cout << "Maximum Read (with copy) Time: " << maxReadTime << " ns" << std::endl;
    std::cout << "Maximum Write Time: " << maxWriteTime << " ns\n" << std::endl;

    // Perform assertions using GTest

    // reading
    ASSERT_LT(averageReadTime, READ_T_AVRG_THRESH);
//    ASSERT_LT(maxReadTime, READ_T_MAX_THRESH);

    // writing
    ASSERT_LT(averageWriteTime, WRITE_T_AVRG_THRESH);
//    ASSERT_LT(maxWriteTime, WRITE_T_MAX_THRESH);


    // satisfying the expected performance
}

//int main(int argc, char** argv) {

//    // This example runs performance benchmarking of writing and reading functions
//    // of SharsorIPCpp for all the supported data types (bool, int, float, double) and,
//    // additionally, also for the StringTensor wrapper. Since the employed methods are
//    // exactly the same between Server and Client, here we only employ servers.
//    // The used memory layout is the default one (set in SharsorIPCpp/DTypes.hpp).

//    ::testing::GTEST_FLAG(filter) =
//            "JournalTest.TestJournal";

//    ::testing::GTEST_FLAG(filter) += ":PerfTestBool.WriteReadBenchmark";
////    ::testing::GTEST_FLAG(filter) += ":StringTensorWrite.WriteReadStrTensorBenchmark";

//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();

//}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

