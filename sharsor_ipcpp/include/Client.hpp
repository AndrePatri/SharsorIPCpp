#pragma once

#include <Eigen/Dense>

class Client {
public:
    Client(std::size_t rows, std::size_t cols);
    void readMemory();
private:
    std::size_t rows_;
    std::size_t cols_;
};
