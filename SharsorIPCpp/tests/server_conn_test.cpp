#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <csignal>
#include <thread>
#include <chrono>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Client.hpp>

#include <SharsorIPCpp/Journal.hpp>

#include <test_utils.hpp>

int N_ITER = 10;
int N_ROWS = 4;
int N_COLS = 7;
int BLOCK_SIZE = 3;

int TENSOR_INCREMENT = 4;

using namespace SharsorIPCpp;

using VLevel = Journal::VLevel;

static std::string name_space = "ConnectionTests";

static Journal journal("ConnectionTestsP1");

class ServerWritesInt : public ::testing::Test {
protected:

    ServerWritesInt() :
                   server_ptr(new Server<int>(
                                     N_ROWS, N_COLS,
                                     "SharsorInt", name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   tensor(Tensor<int>::Zero(N_ROWS,
                                            N_COLS)){

        // init matrix with incremental values
        int counter = 1;
        for (int j = 0; j < N_COLS; ++j) {
            for (int i = 0; i < N_ROWS; ++i) {
                tensor(i, j) = counter++;
            }
        }

        server_ptr->run();

    }

    void SetUp() override {

        // Initialization code (if needed)

    }

    void TearDown() override {

        server_ptr->close();

        // Cleanup code (if needed)

    }

    void updateData() {

      tensor.array() += TENSOR_INCREMENT; // incremening copy

      // writing only a block
      server_ptr->writeTensor(tensor.block(0, 0,
                                           BLOCK_SIZE, BLOCK_SIZE),
                              0, 0);

      std::string message = std::string("Incrementing data block of size (") +
                            std::to_string(BLOCK_SIZE) + std::string("x") + std::to_string(BLOCK_SIZE) +
                            std::string(") ") +
                            std::string("at position (0, 0) by ") + std::to_string(TENSOR_INCREMENT) +
                            std::string("\nN. clients connected: ") +
                            std::to_string(server_ptr->getNClients());

      journal.log("updateData", message, LogType::INFO);

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    Server<int>::UniquePtr server_ptr;

    Tensor<int> tensor;

};

class ServerWritesBool : public ::testing::Test {
protected:

    ServerWritesBool() :
                   server_ptr(new Server<bool>(
                                     N_ROWS, N_COLS,
                                     "SharsorBool", name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   tensor(Tensor<bool>::Zero(N_ROWS,
                                            N_COLS)){

        server_ptr->run();

    }

    void SetUp() override {

        // Initialization code (if needed)

    }

    void TearDown() override {

        server_ptr->close();

        // Cleanup code (if needed)

    }

    void updateData() {

      Tensor<bool> myData(N_ROWS - 2, N_COLS - 2);
      myData.setRandom();

      std::string message = std::string("Randomizing data block of size (") +
                            std::to_string(N_ROWS - 2) + std::string("x") + std::to_string(N_COLS - 2) +
                            std::string(") ") +
                            std::string("at position (1, 1).") +
                            std::string("\nN. clients connected: ") +
                            std::to_string(server_ptr->getNClients());

      journal.log("updateData", message, LogType::INFO);

      // writing only a block
      server_ptr->writeTensor(myData,
                              1, 1);

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    Server<bool>::UniquePtr server_ptr;

    Tensor<bool> tensor;

};

class ServerWritesFloat : public ::testing::Test {
protected:

    ServerWritesFloat() :
                   server_ptr(new Server<float>(
                                     N_ROWS, N_COLS,
                                     "SharsorFloat", name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   tensor(Tensor<float>::Zero(N_ROWS,
                                            N_COLS)){

        server_ptr->run();

    }

    void SetUp() override {

        // Initialization code (if needed)

    }

    void TearDown() override {

        server_ptr->close();

        // Cleanup code (if needed)

    }

    void updateData() {

      Tensor<float> myData(N_ROWS - 2, N_COLS - 2);
      myData.setRandom();

      std::string message = std::string("Randomizing data block of size (") +
                            std::to_string(N_ROWS - 2) + std::string("x") + std::to_string(N_COLS - 2) +
                            std::string(") ") +
                            std::string("at position (1, 1).") +
                            std::string("\nN. clients connected: ") +
                            std::to_string(server_ptr->getNClients());

      journal.log("updateData", message, LogType::INFO);

      // writing only a block
      server_ptr->writeTensor(myData,
                              1, 1);

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    Server<float>::UniquePtr server_ptr;

    Tensor<float> tensor;

};

TEST_F(ServerWritesInt, ServerWriteIntBlock) {

    check_comp_type(journal);

    journal.log("ServerWritesInt", "\n Starting to write ...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < N_ITER; ++i) {

        updateData();
    }
}

TEST_F(ServerWritesBool, ServerWriteBoolRandBlock) {

    check_comp_type(journal);

    journal.log("ServerWritesBool", "\n Starting to randomize ...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < N_ITER; ++i) {

        updateData();
    }
}

TEST_F(ServerWritesFloat, ServerWritesRandFloatBlock) {

    check_comp_type(journal);

    journal.log("ServerWritesFloat", "\n Starting to randomize ...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < N_ITER; ++i) {

        updateData();
    }
}

int main(int argc, char** argv) {

//    ::testing::GTEST_FLAG(filter) =
//        ":ServerWritesInt.ServerWriteIntBlock";

//    ::testing::GTEST_FLAG(filter) +=
//        ":ServerWritesBool.ServerWriteBoolRandBlock";

//    ::testing::GTEST_FLAG(filter) +=
//        ":ServerWritesFloat.ServerWriteFloatRandBlock";

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
