#include "Server.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <cstring> // for std::strerror()
#include <typeinfo>

using namespace SharsorIPCpp;

Server::Server(int n_rows,
               int n_cols,
               std::string memname,
               std::string name_space)
    : n_rows(n_rows),
      n_cols(n_cols),
      _shared_mem_name(memname),
      _namespace{name_space},
      _tensor_view(nullptr,
                   n_rows,
                   n_rows),
      _journal(Journal(GetThisName()))
{

    std::size_t data_size = sizeof(float) * n_rows * n_cols;

    Tensor _tensor_copy = Tensor::Zero(n_rows, n_cols);

    // Create shared memory
    _shm_fd = shm_open(_shared_mem_name.c_str(),
                       O_CREAT | O_RDWR,
                       S_IRUSR | S_IWUSR);

    if (_shm_fd == -1) {

        throw std::runtime_error("Cannot create shared memory: " +
                                 std::string(std::strerror(errno)));
    }

    // Set size
    if (ftruncate(_shm_fd, data_size) == -1) {

        throw std::runtime_error("Cannot set shared memory size: " +
                                 std::string(std::strerror(errno)));

    }

    // Map the shared memory
    float* matrix_data = static_cast<float*>(mmap(nullptr, data_size,
                                                    PROT_READ | PROT_WRITE,
                                                    MAP_SHARED, _shm_fd, 0));
    if (matrix_data == MAP_FAILED) {

        throw std::runtime_error("Cannot map shared memory: " +
                                 std::string(std::strerror(errno)));
    }

    new (&_tensor_view) MMap(matrix_data, n_rows, n_cols);
}

Server::~Server() {

    shm_unlink(_shared_mem_name.c_str());

}

std::string Server::GetThisName()
{
    const std::type_info& info = typeid(*this);

    return std::string(info.name());
}

void Server::writeMemory(const Tensor& data) {

    if(data.rows() != n_rows || data.cols() != n_cols) {

        throw std::runtime_error("Data dimensions mismatch");

    }

    _tensor_view.block(0, 0, n_rows, n_cols) = data;
}

const MMap& Server::getTensorView() {

    return _tensor_view;

}

const Tensor& Server::getTensorCopy() {

    _tensor_copy = _tensor_view;

    return _tensor_copy;

}
