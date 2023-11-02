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

// normal tensor API
template <typename P>
class ConsistencyChecks : public ::testing::Test {
protected:

    using ScalarType = typename P::type;
    static const int layout = P::layout;

    ConsistencyChecks() :
                   client_ping_ptr(new Client<ScalarType, layout>("SharsorIPCpp_ping",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)),
                   client_pong_ptr(new Client<ScalarType, layout>("SharsorIPCpp_pong",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)),
                   client_terminate_ptr(new Client<bool>("SharsorIPCpp_terminate",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)),
                   client_flag_ptr(new Client<bool>("SharsorIPCpp_flag",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)){

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

            while (!client_terminate_ptr->readTensor(terminate)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

            while (!client_flag_ptr->readTensor(flag)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

            while (!flag(0,0)){

                while (!client_terminate_ptr->readTensor(terminate)) {

                    // try again
                    std::this_thread::sleep_for(std::chrono::microseconds(1));

                }
                if (terminate(0, 0)) {

                    return;
                }

                client_flag_ptr->readTensor(flag); // reads flag from client

                std::this_thread::sleep_for(std::chrono::microseconds(1)); // wait

            }

            // server has written new ping memory, let's read it

            while (!client_ping_ptr->readTensor(data_read, 0, 0)) {

                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }

            // and write it on the pong memory

            while (!client_pong_ptr->writeTensor(data_read, 0, 0)) {

            }

            // signaling the server that the read/write operation was completed
            flag(0, 0) = false;
            while (!client_flag_ptr->writeTensor(flag)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

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

// view API (this client uses the normal tensor API,
// while the server will use the view API)
template <typename P>
class ViewsConsistencyChecks : public ::testing::Test {
protected:

    using ScalarType = typename P::type;
    static const int layout = P::layout;

    ViewsConsistencyChecks() :
                   client_ping_ptr(new Client<ScalarType, layout>("SharsorIPCpp_view_ping",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)),
                   client_pong_ptr(new Client<ScalarType, layout>("SharsorIPCpp_view_pong",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)),
                   client_terminate_ptr(new Client<bool>("SharsorIPCpp_view_terminate",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)),
                   client_flag_ptr(new Client<bool>("SharsorIPCpp_view_flag",
                                     name_space + getTypeAsString<ScalarType>() + std::to_string(layout),
                                     false,
                                     VLevel::V3)){

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

            while (!client_terminate_ptr->readTensor(terminate)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

            while (!client_flag_ptr->readTensor(flag)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

            while (!flag(0,0)){

                while (!client_terminate_ptr->readTensor(terminate)) {

                    // try again
                    std::this_thread::sleep_for(std::chrono::microseconds(1));

                }
                if (terminate(0, 0)) {

                    return;
                }

                client_flag_ptr->readTensor(flag); // reads flag from client

                std::this_thread::sleep_for(std::chrono::microseconds(1)); // wait

            }

            // server has written new ping memory, let's read it

            while (!client_ping_ptr->readTensor(data_read, 0, 0)) {

                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }

            // and write it on the pong memory

            while (!client_pong_ptr->writeTensor(data_read, 0, 0)) {

            }

            // signaling the server that the read/write operation was completed
            flag(0, 0) = false;
            while (!client_flag_ptr->writeTensor(flag)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

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

class StringTensorCheck : public ::testing::Test {
protected:

    StringTensorCheck() :

                    string_t_ping_ptr(new StringTensor<StrClient>(
                                     "SharedStrTensor_ping",
                                     name_space,
                                     false,
                                     VLevel::V3)),
                    string_t_pong_ptr(new StringTensor<StrClient>(
                                     "SharedStrTensor_pong",
                                     name_space,
                                     false,
                                     VLevel::V3)),
                    client_terminate_ptr(new Client<bool>(
                                   "SharedStrTensor_terminate",
                                   name_space,
                                   false,
                                   VLevel::V3)),
                    client_flag_ptr(new Client<bool>(
                                   "SharedStrTensor_flag",
                                   name_space,
                                   false,
                                   VLevel::V3))
    {

        string_t_ping_ptr->run();
        string_t_pong_ptr->run();
        client_terminate_ptr->attach();
        client_flag_ptr->attach();

        getDataFromServer();

        data_read.resize(length);

    }

    void getDataFromServer() {

        length = string_t_ping_ptr->getLength();

        // reads initializations from Server

        terminate = Tensor<bool>::Zero(client_terminate_ptr->getNRows(),
                                       client_terminate_ptr->getNCols());
        flag = Tensor<bool>::Zero(client_flag_ptr->getNRows(),
                                  client_flag_ptr->getNCols());

        client_terminate_ptr->readTensor(terminate);
        client_flag_ptr->readTensor(flag);


    }

    void SetUp() override {

    }

    void TearDown() override {

        string_t_ping_ptr->close();
        string_t_pong_ptr->close();
        client_terminate_ptr->close();
        client_flag_ptr->close();

    }

    void Run() {

        while (!terminate(0, 0)) { // exit if received a signal from server

            while (!client_terminate_ptr->readTensor(terminate)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

            while (!client_flag_ptr->readTensor(flag)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

            while (!flag(0,0)){

                while (!client_terminate_ptr->readTensor(terminate)) {

                    // try again
                    std::this_thread::sleep_for(std::chrono::microseconds(1));

                }
                if (terminate(0, 0)) {

                    return;
                }

                client_flag_ptr->readTensor(flag); // reads flag from client

                std::this_thread::sleep_for(std::chrono::microseconds(1)); // wait

            }

            // server has written new ping memory, let's read it

            while (!string_t_ping_ptr->read(data_read, 0)) {

                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }

            // and write it on the pong memory

            while (!string_t_pong_ptr->write(data_read, 0)) {

            }

            // signaling the server that the read/write operation was completed
            flag(0, 0) = false;
            while (!client_flag_ptr->writeTensor(flag)) {

                // try again
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            }

        }

    }

    int length;
    int iterations;
    typename StringTensor<StrClient>::UniquePtr string_t_ping_ptr,
                                                string_t_pong_ptr;


    typename Client<bool>::UniquePtr client_terminate_ptr,
                                client_flag_ptr;

    std::vector<std::string> data_read;

    Tensor<bool> terminate = Tensor<bool>::Zero(1, 1); // flags to false
    Tensor<bool> flag = Tensor<bool>::Zero(1, 1);

    std::vector<bool> checks;

};

// Register the tests for normal tensor API
TYPED_TEST_SUITE_P(ConsistencyChecks);

TYPED_TEST_P(ConsistencyChecks, CheckConsistencyDTypes) {

   check_comp_type(journal);

   this->Run(); // run round-trip checks with client

}

REGISTER_TYPED_TEST_SUITE_P(ConsistencyChecks, CheckConsistencyDTypes);

INSTANTIATE_TYPED_TEST_SUITE_P(ConsistencyCheck, ConsistencyChecks, MyTypes);

// Register the tests for view API
TYPED_TEST_SUITE_P(ViewsConsistencyChecks);

TYPED_TEST_P(ViewsConsistencyChecks, CheckConsistencyDTypes) {

   check_comp_type(journal);

   this->Run(); // run round-trip checks with client

}

REGISTER_TYPED_TEST_SUITE_P(ViewsConsistencyChecks, CheckConsistencyDTypes);

INSTANTIATE_TYPED_TEST_SUITE_P(ConsistencyViewCheck, ViewsConsistencyChecks, MyTypes);

// StringTensor checks
TEST_F(StringTensorCheck, StringTensorChecks) {

    check_comp_type(journal);

    this->Run();

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
