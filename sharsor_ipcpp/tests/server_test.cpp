#include "Server.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

using namespace SharsorIPCpp;

int main() {
    int rows = 100;
    int cols = 60;
    Server server(rows, cols, "SharedMemTest");

    std::cout << "Server: Running. Press ENTER to exit...\n";

    Tensor tensor_copy = Tensor::Zero(rows, cols);

    // init. here for profiling
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    while (true) {

        // Generate new random data
        Tensor data = Tensor::Random(rows, cols);

        // Write new data
        start = std::chrono::high_resolution_clock::now();

        server.writeMemory(data);

        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Server: Write took " << duration.count() << "us\n";

        // Read data
        start = std::chrono::high_resolution_clock::now();

        tensor_copy = server.getTensorCopy();

        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Server: Read took " << duration.count() << "us\n";

        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Adjust the rate as per requirement

        std::cout << tensor_copy  << std::endl;
//        std::cout << server.getTensorView() << std::endl;
        std::cout << "###############" << std::endl;
    }

    return 0;
}
