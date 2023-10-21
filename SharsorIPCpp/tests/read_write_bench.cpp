#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <csignal>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/StringTensor.hpp>

#include <SharsorIPCpp/Journal.hpp>

#include <test_utils.hpp>

int N_ITERATIONS = 10000000;
int N_ITERATIONS_STR = 10000000;

int STR_TENSOR_LENGTH = 100;

using namespace SharsorIPCpp;

using VLevel = Journal::VLevel;

static std::string name_space = "ServerTst";

static Journal journal("ServerTests");

class JournalTest: public ::testing::Test {
protected:
    JournalTest(){

    }

    void SetUp() override {

    }

    void TearDown() override {

    }
};

class ServerTestDouble : public ::testing::Test {
protected:

    ServerTestDouble() : rows(100),
                   cols(60),
                   iterations(N_ITERATIONS),
                   server_ptr(new Server<double>(rows, cols,
                                     "SharsorDouble", name_space,
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
    Server<double>::UniquePtr server_ptr;
    Tensor<double> tensor_copy;

};

class ServerTestFloat : public ::testing::Test {
protected:

    ServerTestFloat() : rows(100),
                   cols(60),
                   iterations(N_ITERATIONS),
                   server_ptr(new Server<float>(rows, cols,
                                     "SharsorFloat", name_space,
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
    Server<float>::UniquePtr server_ptr;
    Tensor<float> tensor_copy;

};

class ServerTestInt : public ::testing::Test {
protected:
    ServerTestInt() : rows(100),
                   cols(60),
                   iterations(N_ITERATIONS),
                   server_ptr(new Server<int>(rows, cols,
                                     "SharsorInt", name_space,
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
    Server<int>::UniquePtr server_ptr;
    Tensor<int> tensor_copy;

};

class ServerTestBool : public ::testing::Test {
protected:
    ServerTestBool() : rows(100),
                   cols(60),
                   iterations(N_ITERATIONS),
                   server_ptr(new Server<bool>(rows, cols,
                                     "SharsorBool", name_space,
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
    Server<bool>::UniquePtr server_ptr;
//    Server<bool> server;
    Tensor<bool> tensor_copy;

};

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

TEST_F(JournalTest, TestJournal) {

    journal.log("DoSomething", "A warning occurred. You can probably ignore this.",
                Journal::LogType::WARN);
    journal.log("DoSomething", "This is some info.",
                Journal::LogType::INFO);
    journal.log("DoSomething", "These are statistics.",
                Journal::LogType::STAT);
    journal.log("DoSomething", "An exception without throw occurred!!",
                Journal::LogType::EXCEP);

    std::cout << ""  << std::endl;

    EXPECT_THROW({
            journal.log("DoSomething", "An exception with throw occurred!!",
                        Journal::LogType::EXCEP,
                        true);
        },
        std::runtime_error) << "Expected a std::runtime_error to be thrown.";

}

TEST_F(ServerTestBool, WriteReadBenchmark) {

    check_comp_type(journal);

    double READ_T_MAX_THRESH =  10000000; // [nanoseconds], maximum allowed read time
    double WRITE_T_MAX_THRESH = 10000000; // [nanoseconds], maximum allowed read time
    double READ_T_AVRG_THRESH =   1000; // [nanoseconds]
    double WRITE_T_AVRG_THRESH =  1000; // [nanoseconds]

    std::vector<double> readTimes;
    std::vector<double> writeTimes;

    journal.log("ServerTestBool", "\nBenchmarking performance with bool type...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < iterations; ++i) {

        Tensor<bool> myData(rows, cols);
        myData.setRandom(); // we generate a random tensor of the right size

        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        server_ptr->writeTensor(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read a copy of the tensor
        auto startRead = std::chrono::high_resolution_clock::now();
        server_ptr->readTensor(tensor_copy);
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);
    }

    journal.log("ServerTestBool", "\nrunning post-processing steps...\n",
                Journal::LogType::STAT);

    // some post-processing
    double averageReadTime = 0;
    double averageWriteTime = 0;
    double maxReadTime = std::numeric_limits<double>::min();
    double maxWriteTime = std::numeric_limits<double>::min();

    for (int i = 0; i < iterations; ++i) {
        averageReadTime += readTimes[i];
        averageWriteTime += writeTimes[i];

        if (readTimes[i] > maxReadTime) {
            maxReadTime = readTimes[i];
        }

        if (writeTimes[i] > maxWriteTime) {
            maxWriteTime = writeTimes[i];
        }
    }

    averageReadTime /= iterations;
    averageWriteTime /= iterations;

    std::cout << "Number of performed iterations: " << iterations << std::endl;
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

TEST_F(ServerTestInt, WriteReadBenchmark) {

    check_comp_type(journal);

    double READ_T_MAX_THRESH =  10000000; // [nanoseconds], maximum allowed read time
    double WRITE_T_MAX_THRESH = 10000000; // [nanoseconds], maximum allowed read time
    double READ_T_AVRG_THRESH =   2300; // [nanoseconds]
    double WRITE_T_AVRG_THRESH =  2300; // [nanoseconds]

    std::vector<double> readTimes;
    std::vector<double> writeTimes;

    journal.log("ServerTestInt", "\nBenchmarking performance with int type...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < iterations; ++i) {

        Tensor<int> myData(rows, cols);
        myData.setRandom(); // we generate a random tensor of the right size

        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        server_ptr->writeTensor(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read a copy of the tensor
        auto startRead = std::chrono::high_resolution_clock::now();
        server_ptr->readTensor(tensor_copy);
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);
    }

    journal.log("ServerTestInt", "\nrunning post-processing steps...\n",
                Journal::LogType::STAT);

    // some post-processing
    double averageReadTime = 0;
    double averageWriteTime = 0;
    double maxReadTime = std::numeric_limits<double>::min();
    double maxWriteTime = std::numeric_limits<double>::min();

    for (int i = 0; i < iterations; ++i) {
        averageReadTime += readTimes[i];
        averageWriteTime += writeTimes[i];

        if (readTimes[i] > maxReadTime) {
            maxReadTime = readTimes[i];
        }

        if (writeTimes[i] > maxWriteTime) {
            maxWriteTime = writeTimes[i];
        }
    }

    averageReadTime /= iterations;
    averageWriteTime /= iterations;

    std::cout << "Number of performed iterations: " << iterations << std::endl;
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

TEST_F(ServerTestFloat, WriteReadBenchmark) {

    check_comp_type(journal);

    double READ_T_MAX_THRESH =  10000000; // [nanoseconds], maximum allowed read time
    double WRITE_T_MAX_THRESH = 10000000; // [nanoseconds], maximum allowed read time
    double READ_T_AVRG_THRESH =   2500; // [nanoseconds]
    double WRITE_T_AVRG_THRESH =  2500; // [nanoseconds]

    std::vector<double> readTimes;
    std::vector<double> writeTimes;

    journal.log("ServerTestInt", "\nBenchmarking performance with float type...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < iterations; ++i) {

        Tensor<float> myData(rows, cols);
        myData.setRandom(); // we generate a random tensor of the right size

        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        server_ptr->writeTensor(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read a copy of the tensor
        auto startRead = std::chrono::high_resolution_clock::now();
        server_ptr->readTensor(tensor_copy);
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);
    }

    journal.log("ServerTestInt", "\nrunning post-processing steps...\n",
                Journal::LogType::STAT);

    // some post-processing
    double averageReadTime = 0;
    double averageWriteTime = 0;
    double maxReadTime = std::numeric_limits<double>::min();
    double maxWriteTime = std::numeric_limits<double>::min();

    for (int i = 0; i < iterations; ++i) {
        averageReadTime += readTimes[i];
        averageWriteTime += writeTimes[i];

        if (readTimes[i] > maxReadTime) {
            maxReadTime = readTimes[i];
        }

        if (writeTimes[i] > maxWriteTime) {
            maxWriteTime = writeTimes[i];
        }
    }

    averageReadTime /= iterations;
    averageWriteTime /= iterations;

    std::cout << "Number of performed iterations: " << iterations << std::endl;
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

TEST_F(ServerTestDouble, WriteReadBenchmark) {

    check_comp_type(journal);

    double READ_T_MAX_THRESH =  10000000; // [nanoseconds], maximum allowed read time
    double WRITE_T_MAX_THRESH = 10000000; // [nanoseconds], maximum allowed read time
    double READ_T_AVRG_THRESH =   5000; // [nanoseconds]
    double WRITE_T_AVRG_THRESH =  5000; // [nanoseconds]

    std::vector<double> readTimes;
    std::vector<double> writeTimes;

    journal.log("ServerTestDouble", "\nBenchmarking performance with double type...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < iterations; ++i) {

        Tensor<double> myData(rows, cols);
        myData.setRandom(); // we generate a random tensor of the right size

        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        server_ptr->writeTensor(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read a copy of the tensor
        auto startRead = std::chrono::high_resolution_clock::now();
        server_ptr->readTensor(tensor_copy);
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);
    }

    journal.log("ServerTestDouble", "\nrunning post-processing steps...\n",
                Journal::LogType::STAT);

    // some post-processing
    double averageReadTime = 0;
    double averageWriteTime = 0;
    double maxReadTime = std::numeric_limits<double>::min();
    double maxWriteTime = std::numeric_limits<double>::min();

    for (int i = 0; i < iterations; ++i) {
        averageReadTime += readTimes[i];
        averageWriteTime += writeTimes[i];

        if (readTimes[i] > maxReadTime) {
            maxReadTime = readTimes[i];
        }

        if (writeTimes[i] > maxWriteTime) {
            maxWriteTime = writeTimes[i];
        }
    }

    averageReadTime /= iterations;
    averageWriteTime /= iterations;

    std::cout << "Number of performed iterations: " << iterations << std::endl;
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

TEST_F(StringTensorWrite, WriteReadStrTensorBenchmark) {

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

int main(int argc, char** argv) {

    // Set the GTEST_FILTER environment variable to specify the tests to run.
    // You can list multiple tests or test suites separated by a colon.
    // This example runs two test cases: ServerTestDouble.WriteReadBenchmark and AnotherTestSuite.*

    ::testing::GTEST_FLAG(filter) =
            "JournalTest.TestJournal";

    ::testing::GTEST_FLAG(filter) += ":ServerTestDouble.WriteReadBenchmark";
    ::testing::GTEST_FLAG(filter) += ":ServerTestFloat.WriteReadBenchmark";
    ::testing::GTEST_FLAG(filter) += ":ServerTestInt.WriteReadBenchmark";
    ::testing::GTEST_FLAG(filter) += ":ServerTestBool.WriteReadBenchmark";
//    ::testing::GTEST_FLAG(filter) += ":StringTensorWrite.WriteReadStrTensorBenchmark";

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}

