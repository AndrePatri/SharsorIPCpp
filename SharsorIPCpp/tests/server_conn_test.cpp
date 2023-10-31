#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <csignal>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/StringTensor.hpp>

#include <SharsorIPCpp/Journal.hpp>

#include <test_utils.hpp>

int N_ITER = 10;
int N_ITER_STR = 10;

int N_ROWS = 100;
int N_COLS = 60;
int BLOCK_SIZE = 3;

int TENSOR_INCREMENT = 1;

size_t STR_TENSOR_LENGTH = 10;

using namespace SharsorIPCpp;

using VLevel = Journal::VLevel;

static std::string name_space = "ConnectionTests";

static Journal journal("ConnectionTestsP1");

class ServerWritesInt : public ::testing::Test {
protected:

    ServerWritesInt() :
                   server_ptr(new Server<int, SharsorIPCpp::MemLayoutDefault>(
                                     N_ROWS, N_COLS,
                                     "SharsorInt", name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   tensor(Tensor<int, SharsorIPCpp::MemLayoutDefault>::Zero(N_ROWS,
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

      Tensor<int, SharsorIPCpp::MemLayoutDefault> myData(N_ROWS - 2, N_COLS - 2);
      Tensor<int, SharsorIPCpp::MemLayoutDefault> myDataFull(N_ROWS, N_COLS);
      myData.setRandom();

      std::string message = std::string("Randomizing data block of size (") +
                            std::to_string(N_ROWS - 2) + std::string("x") + std::to_string(N_COLS - 2) +
                            std::string(") ") +
                            std::string("at position (1, 1).") +
                            std::string("\nN. clients connected: ") +
                            std::to_string(server_ptr->getNClients());

      journal.log("updateData", message, LogType::INFO);

      std::cout << "Writing block:" << std::endl;
      std::cout << myData << std::endl;

      // writing only a block
      server_ptr->writeTensor(myData,
                              1, 1);

      server_ptr->readTensor(myDataFull);

      std::cout << "Full tensor:" << std::endl;
      std::cout << myDataFull << std::endl;
      std::cout << "###########" << std::endl;

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    Server<int, SharsorIPCpp::MemLayoutDefault>::UniquePtr server_ptr;

    Tensor<int, SharsorIPCpp::MemLayoutDefault> tensor;

};

class ServerWritesBool : public ::testing::Test {
protected:

    ServerWritesBool() :
                   server_ptr(new Server<float, SharsorIPCpp::MemLayoutDefault>(
                                     N_ROWS, N_COLS,
                                     "SharsorBool", name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   tensor(Tensor<float, SharsorIPCpp::MemLayoutDefault>::Zero(N_ROWS,
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

      Tensor<float, SharsorIPCpp::MemLayoutDefault> myData(N_ROWS, N_COLS);
      Tensor<float, SharsorIPCpp::MemLayoutDefault> myDataFull(N_ROWS, N_COLS);
      myData.setRandom();

      std::string message = std::string("Randomizing data block of size (") +
                            std::to_string(N_ROWS - 2) + std::string("x") + std::to_string(N_COLS - 2) +
                            std::string(") ") +
                            std::string("at position (1, 1).") +
                            std::string("\nN. clients connected: ") +
                            std::to_string(server_ptr->getNClients());

      journal.log("updateData", message, LogType::INFO);

      std::cout << "Writing block:" << std::endl;
      std::cout << myData << std::endl;

      // writing only a block
      server_ptr->writeTensor(myData,
                              0, 0);

      server_ptr->readTensor(myDataFull);

      std::cout << "Full tensor:" << std::endl;
      std::cout << myDataFull << std::endl;
      std::cout << "###########" << std::endl;

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    Server<float, SharsorIPCpp::MemLayoutDefault>::UniquePtr server_ptr;

    Tensor<float, SharsorIPCpp::MemLayoutDefault> tensor;

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
                   str_vec(STR_TENSOR_LENGTH),
                   str_vec_check(STR_TENSOR_LENGTH){

        str_vec[0] = "MaremmaMaiala";

        str_vec[1] = "?=^/$£*ç°§_";

        str_vec[2] = "Scibidibi97";

        str_vec[6] = "Joint_dummy2";

        str_vec[9] = "Sbirulina";

        string_t_ptr->run();

    }

    void SetUp() override {

    }

    void TearDown() override {

        string_t_ptr->close();

    }

    void updateData() {

        bool succes_write = string_t_ptr->write(str_vec, 0);

        bool success_check = string_t_ptr->read(str_vec_check, 0);

        str_vec[random_int(str_vec.size() - 1)] = random_string(5); // random
        // update at random index

        std::string delimiter = ", ";

        std::string result = std::accumulate(str_vec.begin(), str_vec.end(),
                                             std::string(),
                                             [&delimiter](const std::string& a, const std::string& b) {
                                                 return a.empty() ? b : a + delimiter + b;
                                             });

        journal.log("StringTensorRead",
                    std::string("\nWritten vector:\n") + result + std::string("\n"),
                    Journal::LogType::STAT);

        std::this_thread::sleep_for(std::chrono::seconds(5));

    }

    StringTensor<StrServer>::UniquePtr string_t_ptr;

    std::vector<std::string> str_vec;

    std::vector<std::string> str_vec_check;

};

TEST_F(StringTensorWrite, StringTensorCheck) {

    check_comp_type(journal);

    journal.log("StringTensorWrite", "\n Starting to write string tensor...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < N_ITER_STR; ++i) {

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

TEST_F(ServerWritesInt, ServerWriteIntRandBlock) {

    check_comp_type(journal);

    journal.log("ServerWritesInt", "\n Starting to randomize ...\n",
                Journal::LogType::STAT);

    for (int i = 0; i < N_ITER; ++i) {

        updateData();
    }
}

int main(int argc, char** argv) {

//    ::testing::GTEST_FLAG(filter) =
//        ":ServerWritesInt.ServerWriteIntRandBlock";

    ::testing::GTEST_FLAG(filter) =
        ":ServerWritesBool.ServerWriteBoolRandBlock";

//    ::testing::GTEST_FLAG(filter) =
//        ":ServerWritesFloat.ServerWritesRandFloatBlock";

//    ::testing::GTEST_FLAG(filter) =
//        ":StringTensorWrite.StringTensorCheck";

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
