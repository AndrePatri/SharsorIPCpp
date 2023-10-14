#include "Server.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <iostream>
#include <Eigen/Dense>

using namespace boost::interprocess;
using Eigen::Map;
using Eigen::MatrixXf;

Server::Server(std::size_t rows, std::size_t cols)
    : rows_(rows), cols_(cols)
    // Check if shared memory already exists and remove it
    if(shared_memory_object::remove("MySharedMemory")) {
        std::cout << "Server: Old shared memory removed." << std::endl;
    }

    // Create shared memory
    managed_shared_memory shm(create_only, "MySharedMemory", sizeof(float) * rows_ * cols_ * 2); // Sufficient space

    // Allocate array in the shared memory and get the raw pointer
    float* myArray = shm.construct<float>("MyArray")[rows_ * cols_]();

    // Initialize the Eigen::Map after having the valid pointer
    memory_matrix_ = Map<MatrixXf, 0, Eigen::Stride<0,0>>(myArray, rows_, cols_);
}

Server::~Server() {
    // Removal of the shared memory should be done when no longer needed by any process
    // If this is the owner process, it might be done here.
    shared_memory_object::remove("MySharedMemory");
}

void Server::fillMemory() {
    // Directly fill the memory_matrix_ with random values
    memory_matrix_ = MatrixXf::Random(rows_, cols_);
    std::cout << "Server: Memory filled with random values:\n" << memory_matrix_ << std::endl;
}

void Server::writeMemory(const MatrixXf& data) {
    if(data.rows() != rows_ || data.cols() != cols_) {
        throw std::runtime_error("Data dimensions mismatch");
    }
    memory_matrix_ = data;
}

void Server::run() {
    // Implementation of run...
}
