#include "Server.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <cstdlib>

using namespace SharsorIPCpp;

int main() {
    int rows = 100; // Specify the number of rows in the matrix
    int cols = 60; // Specify the number of columns in the matrix
    int iterations = 100000; // Number of iterations to perform

    // Create a Server instance
    Server server(rows, cols);

    // Initialize a vector to store read and write times
    std::vector<double> readTimes;
    std::vector<double> writeTimes;

    const MMap& tensorView = server.getTensorView();

    Tensor tensor_copy = Tensor::Zero(rows, cols);

    for (int i = 0; i < iterations; ++i) {
        // Generate a random Eigen MatrixXf with the same dimensions as the shared memory
        Tensor myData(rows, cols);
        myData.setRandom(); // Fills the matrix with random values

        // Measure the write time
        auto startWrite = std::chrono::high_resolution_clock::now();
        server.writeMemory(myData);
        auto endWrite = std::chrono::high_resolution_clock::now();
        double writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endWrite - startWrite).count();
        writeTimes.push_back(writeTime);

        // Measure the read time
        auto startRead = std::chrono::high_resolution_clock::now();
        tensor_copy = server.getTensorCopy();
        auto endRead = std::chrono::high_resolution_clock::now();
        double readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endRead - startRead).count();
        readTimes.push_back(readTime);

        // You can perform operations on tensorView as needed

        // Print the shared memory data for the first iteration
//        std::cout << "Shared Memory Data:" << std::endl;
//        std::cout << tensorView.block(0, 0, 1, 1) << std::endl;
    }

    // Calculate and print the average read and write times
    double averageReadTime = 0;
    double averageWriteTime = 0;
    double maxReadTime = std::numeric_limits<double>::min();
    double maxWriteTime = std::numeric_limits<double>::min();

    for (int i = 0; i < iterations; ++i) {
        averageReadTime += readTimes[i];
        averageWriteTime += writeTimes[i];

        if (readTimes[i] > maxReadTime) {
            maxReadTime = readTimes[i];
        }

        if (writeTimes[i] > maxWriteTime) {
            maxWriteTime = writeTimes[i];
        }
    }

    averageReadTime /= iterations;
    averageWriteTime /= iterations;

    std::cout << "Average Read (with copy) Time: " << averageReadTime << " nanoseconds" << std::endl;
    std::cout << "Average Write Time: " << averageWriteTime << " nanoseconds" << std::endl;
    std::cout << "Maximum Read (with copy) Time: " << maxReadTime << " nanoseconds" << std::endl;
    std::cout << "Maximum Write Time: " << maxWriteTime << " nanoseconds" << std::endl;

    return 0;
}

