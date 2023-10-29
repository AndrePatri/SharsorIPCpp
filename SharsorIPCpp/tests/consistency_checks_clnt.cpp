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

    ConsistencyChecks() :
                   client_ping_ptr(new Client<ScalarType, layout>("SharsorIPCpp_ping",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     true,
                                     VLevel::V1)),
                   client_pong_ptr(new Client<ScalarType, layout>("SharsorIPCpp_pong",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     true,
                                     VLevel::V1)),
                   client_terminate_ptr(new Client<bool>("SharsorIPCpp_terminate",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     true,
                                     VLevel::V1)),
                   client_flag_ptr(new Client<bool>("SharsorIPCpp_flag",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     true,
                                     VLevel::V1)){

        // run all clients
        client_ping_ptr->attach();
        client_pong_ptr->attach();
        client_terminate_ptr->attach();
        client_flag_ptr->attach();


        getDataFromServer();

        data_read = Tensor<ScalarType, layout>::Zero(rows, cols);;


    }

    void Run() {

        while (!terminate(0, 0)) { // exit if received a signal from server

            while (!flag(0,0)){

                client_flag_ptr->readTensor(flag); // reads flag from client

                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // wait
            }

            // server has written new ping memory, let's read it

            client_ping_ptr->readTensor(data_read, 0, 0);

            // and write it on the pong memory

            client_pong_ptr->writeTensor(data_read, 0, 0);

            // signaling the server that the read/write operation was completed
            flag(0, 0) = false;
            client_flag_ptr->writeTensor(flag);

            client_terminate_ptr->readTensor(terminate); // updated termination flag from server

        }

    }

    void getDataFromServer() {

        rows = client_ping_ptr->getNRows();
        cols = client_ping_ptr->getNCols();

        // reads initializations from Server

        terminate = Tensor<bool>::Zero(client_terminate_ptr->getNRows(),
                                       client_terminate_ptr->getNCols());
        flag = Tensor<bool>::Zero(client_flag_ptr->getNRows(),
                                  client_flag_ptr->getNCols());

        client_terminate_ptr->readTensor(terminate);
        client_flag_ptr->readTensor(flag);


    }

    void SetUp() override {

        // Initialization code (if needed)
    }

    void TearDown() override {

        // close all clients
        client_ping_ptr->close();
        client_pong_ptr->close();
        client_terminate_ptr->close();
        client_flag_ptr->close();

    }

    int rows;
    int cols;

    typename Client<ScalarType, layout>::UniquePtr client_ping_ptr,
                                                   client_pong_ptr;


    typename Client<bool>::UniquePtr client_terminate_ptr,
                                client_flag_ptr;

    Tensor<ScalarType, layout> data_read;

    Tensor<bool> terminate;
    Tensor<bool> flag;

};

TYPED_TEST_SUITE_P(ConsistencyChecks);

TYPED_TEST_P(ConsistencyChecks, CheckConsistencyDTypes) {

   this->Run(); // run round-trip checks with client

}

// Register the tests
REGISTER_TYPED_TEST_SUITE_P(ConsistencyChecks, CheckConsistencyDTypes);

INSTANTIATE_TYPED_TEST_SUITE_P(My, ConsistencyChecks, MyTypes);

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
