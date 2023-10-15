#include "Server.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>

using namespace SharsorIPCpp;

void check_comp_type()
{
    std::string message;

    #ifdef NDEBUG

        #ifdef _RELWITHDEBINFO

            message = std::string("[Warning]: SharsorIPCpp was compiled in RelWithDebInfo mode. ") +
                    std::string("For meaninful results, you should compile it in Release mode.\n");
        #else

            message = std::string("[Info]: Library was compiled in Release mode. ") +
                std::string("This will ensure meaningful benchmarking results.\n");

        #endif

    #else

        message = std::string("[Warning]: SharsorIPCpp was compiled in Debug mode. ") +
            std::string("For meaninful results, you should compile it in Release mode.\n");

    #endif

    std::cout << message << std::endl;
}

class ServerTest : public ::testing::Test {
protected:
    ServerTest() : rows(100),
                   cols(60),
                   iterations(1000000),
                   server(rows, cols),
                   tensor_copy(rows, cols) {

        // Constructor code (if needed)

    }

    void SetUp() override {

        // Initialization code (if needed)

    }

    void TearDown() override {

        // Cleanup code (if needed)

    }

    int rows;
    int cols;
    int iterations;
    Server server;
    Tensor tensor_copy;

};

TEST_F(ServerTest, WriteReadBenchmark) {

    check_comp_type();

    double READ_T_MAX_THRESH = 60000; // [nanoseconds], maximum allowed read time
    double WRITE_T_MAX_THRESH = 60000; // [nanoseconds], maximum allowed read time
    double READ_T_AVRG_THRESH = 2000; // [nanoseconds]
    double WRITE_T_AVRG_THRESH = 2000; // [nanoseconds]

    std::vector<double> readTimes;
    std::vector<double> writeTimes;

    const MMap& tensorView = server.getTensorView(); // its a reference
    // we only need to get it once

    std::cout << "\nGenerating samples...\n" << std::endl;

    for (int i = 0; i < iterations; ++i) {

        Tensor myData(rows, cols);
        myData.setRandom(); // we generate a random tensor of the right size

        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        server.writeMemory(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // we measure the time to read a copy of the tensor
        auto startRead = std::chrono::high_resolution_clock::now();
        tensor_copy = server.getTensorCopy();
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);
    }

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
    ASSERT_LT(averageReadTime, READ_T_AVRG_THRESH); // we check the maximim
    ASSERT_LT(maxReadTime, READ_T_MAX_THRESH); // we check that we are

    // writing
    ASSERT_LT(averageWriteTime, WRITE_T_AVRG_THRESH); // we check that we are
    ASSERT_LT(maxWriteTime, WRITE_T_MAX_THRESH); // we check that we are


    // satisfying the expected performance
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

