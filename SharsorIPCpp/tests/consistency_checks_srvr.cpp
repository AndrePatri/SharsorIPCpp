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

int N_ITERATIONS = 100000;
int N_ITERATIONS_STR = 100000;

int N_ROWS = 100;
int N_COLS = 60;

int STR_TENSOR_LENGTH = 100;

using namespace SharsorIPCpp;

using VLevel = Journal::VLevel;

static std::string name_space = "ConsistencyTests";

static Journal journal("ConsistencyTests");

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
class ConsistencyChecks : public ::testing::Test {
protected:

    using ScalarType = typename P::type;
    static const int layout = P::layout;

    ConsistencyChecks() : rows(N_ROWS),
                   cols(N_COLS),
                   iterations(N_ITERATIONS),
                   server_ping_ptr(new Server<ScalarType, layout>(rows, cols,
                                     "SharsorIPCpp_ping",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3,
                                     true)),
                   server_pong_ptr(new Server<ScalarType, layout>(rows, cols,
                                     "SharsorIPCpp_pong",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3,
                                     true)),
                   server_terminate_ptr(new Server<bool>(1, 1,
                                     "SharsorIPCpp_terminate",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3,
                                     true)),
                   server_flag_ptr(new Server<bool>(1, 1,
                                     "SharsorIPCpp_flag",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3,
                                     true)),
                   data_written(rows, cols),
                   data_read(rows, cols){

        // run all servers
        server_ping_ptr->run();
        server_pong_ptr->run();
        server_terminate_ptr->run();
        server_flag_ptr->run();

        terminate(0, 0) = false;
        server_terminate_ptr->writeTensor(terminate);

    }

    void Run() {

        for (int i = 0; i < this->iterations; ++i) {

            data_written.setRandom(); // randomize data

            server_ping_ptr->writeTensor(data_written, 0, 0); // writes down the whole random matrix on ping memory

            flag(0, 0) = true;
            while(!server_flag_ptr->writeTensor(flag)) {
                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
             // signals the client the write operation was completed

            while (flag(0, 0)){

                server_flag_ptr->readTensor(flag); // continue reading the flag

                std::this_thread::sleep_for(std::chrono::microseconds(1)); // avoid busy wait loop

            }
            // client sets flag to false -> it has read the data and copied to the pong data

            server_pong_ptr->readTensor(data_read, 0, 0); // reads data from pong memory

            checks.push_back(areEqual<ScalarType>(data_written, data_read));

        }

        terminate(0, 0) = true;
        while(!server_terminate_ptr->writeTensor(terminate)) {

            // try again
            std::this_thread::sleep_for(std::chrono::microseconds(1));

        }

    }

    void SetUp() override {

        // Initialization code (if needed)
    }

    void TearDown() override {

        // close all servers
        server_ping_ptr->close();
        server_pong_ptr->close();
        server_terminate_ptr->close();
        server_flag_ptr->close();

    }

    int rows;
    int cols;
    int iterations;
    typename Server<ScalarType, layout>::UniquePtr server_ping_ptr,
                                                   server_pong_ptr;


    typename Server<bool>::UniquePtr server_terminate_ptr,
                                server_flag_ptr;

    Tensor<ScalarType, layout> data_read, data_written;

    Tensor<bool> terminate = Tensor<bool>::Zero(1, 1); // flags to false
    Tensor<bool> flag = Tensor<bool>::Zero(1, 1);

    std::vector<bool> checks;

};

TYPED_TEST_SUITE_P(ConsistencyChecks);

TYPED_TEST_P(ConsistencyChecks, CheckConsistencyDTypes) {

   this->Run(); // run round-trip checks with client

    int n_failures = 0;

    bool success = allTrue(this->checks, n_failures);

    std::cout << "Test completed. Number of failures: " << n_failures << std::endl;

    ASSERT_TRUE(success);

}

// Register the tests
REGISTER_TYPED_TEST_SUITE_P(ConsistencyChecks, CheckConsistencyDTypes);

INSTANTIATE_TYPED_TEST_SUITE_P(My, ConsistencyChecks, MyTypes);

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
