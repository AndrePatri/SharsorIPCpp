#pragma once

#include <Eigen/Dense>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <stdexcept> // For std::runtime_error

class Server {
public:
    Server(std::size_t rows, std::size_t cols);
    ~Server();

    void writeMemory(const Eigen::MatrixXf& data);
    void run();
    void fillMemory();

private:
    std::size_t rows_;
    std::size_t cols_;

    bool running_{true};

    Eigen::Map<Eigen::MatrixXf> memory_matrix_;
};
