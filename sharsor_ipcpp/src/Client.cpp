#include "Client.hpp"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>

#include <thread>  // For std::this_thread
#include <chrono>  // For std::chrono

using namespace boost::interprocess;
using Eigen::Map;
using Eigen::MatrixXf;

Client::Client(std::size_t rows, std::size_t cols) : rows_(rows), cols_(cols) {}

void Client::readMemory() {
    std::string shmem_name = "MySharedMemory";
    try {
        shared_memory_object shmem(open_only, shmem_name.c_str(), read_only);
        mapped_region region(shmem, read_only);
        void* addr = region.get_address();
        Map<MatrixXf> mat(static_cast<float*>(addr), rows_, cols_);
        std::cout << "Client: Connected! Reading shared memory:\n" << mat << std::endl;
    }
    catch (interprocess_exception& e) {
        std::cerr << "Client: Waiting for server to start...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for 1 second
    }
}
