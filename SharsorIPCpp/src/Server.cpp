#include "Server.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <cstring> // for std::strerror()

using namespace SharsorIPCpp;

Server::Server(int rows, int cols, std::string memname)
    : rows_(rows), cols_(cols), _shared_mem_name(memname),
      tensor_view_(nullptr, rows, cols)
{
    std::size_t data_size = sizeof(float) * rows_ * cols_;

    Tensor tensor_copy_ = Tensor::Zero(rows_, cols_);

    // Create shared memory
    shm_fd_ = shm_open(_shared_mem_name.c_str(), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (shm_fd_ == -1) {
        throw std::runtime_error("Cannot create shared memory: " + std::string(std::strerror(errno)));
    }

    // Set size
    if (ftruncate(shm_fd_, data_size) == -1) {
        throw std::runtime_error("Cannot set shared memory size: " + std::string(std::strerror(errno)));
    }

    // Map the shared memory
    float* matrix_data = static_cast<float*>(mmap(nullptr, data_size,
                                                    PROT_READ | PROT_WRITE,
                                                    MAP_SHARED, shm_fd_, 0));
    if (matrix_data == MAP_FAILED) {
        throw std::runtime_error("Cannot map shared memory: " + std::string(std::strerror(errno)));
    }

    new (&tensor_view_) MMap(matrix_data, rows_, cols_);
}

Server::~Server() {
    shm_unlink(_shared_mem_name.c_str());
}

void Server::writeMemory(const Tensor& data) {
    if(data.rows() != rows_ || data.cols() != cols_) {
        throw std::runtime_error("Data dimensions mismatch");
    }

    tensor_view_.block(0, 0, rows_, cols_) = data;
}

const MMap& Server::getTensorView() {

    return tensor_view_;

}

const Tensor& Server::getTensorCopy() {

    tensor_copy_ = tensor_view_;

    return tensor_copy_;

}
