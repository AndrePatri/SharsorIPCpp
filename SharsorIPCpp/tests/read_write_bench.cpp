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

// Define a structure to hold both the scalar type and the memory layout
template<typename T, int Layout>
struct TypeWithLayout {
    using type = T;
    static const int layout = Layout;
};

// List of types to test against
using MyTypes = ::testing::Types<
    TypeWithLayout<bool, ColMajor>,
    TypeWithLayout<bool, RowMajor>,
    TypeWithLayout<int, ColMajor>,
    TypeWithLayout<int, RowMajor>,
    TypeWithLayout<float, ColMajor>,
    TypeWithLayout<float, RowMajor>,
    TypeWithLayout<double, ColMajor>,
    TypeWithLayout<double, RowMajor>
>;

template <typename P>
class PerfTest : public ::testing::Test {
protected:

    using ScalarType = typename P::type;
    static const int layout = P::layout;

    PerfTest() : rows(100),
                   cols(60),
                   iterations(N_ITERATIONS),
                   server_ptr(new Server<ScalarType, layout>(rows, cols,
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
    typename Server<ScalarType, layout>::UniquePtr server_ptr;
    Tensor<ScalarType, layout> tensor_copy;

};

TYPED_TEST_SUITE_P(PerfTest);

TYPED_TEST_P(PerfTest, WriteReadBenchmark) {

    using ScalarType = typename TestFixture::ScalarType;
    const int layout = TestFixture::layout;

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
        this->server_ptr->write(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read a copy of the tensor
        auto startRead = std::chrono::high_resolution_clock::now();
        this->server_ptr->read(this->tensor_copy, 0, 0);
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

    // reading (max)
//    ASSERT_LT(maxReadTime, READ_T_MAX_THRESH);
//    ASSERT_LT(maxReadTimeView, READ_TV_MAX_THRESH);
//    ASSERT_LT(maxWriteTime, WRITE_T_MAX_THRESH);

}

// Register the tests
REGISTER_TYPED_TEST_SUITE_P(PerfTest, WriteReadBenchmark);
INSTANTIATE_TYPED_TEST_SUITE_P(My, PerfTest, MyTypes);

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
    double READ_T_AVRG_THRESH =   50000; // [nanoseconds]
    double WRITE_T_AVRG_THRESH =  50000; // [nanoseconds]

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

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

