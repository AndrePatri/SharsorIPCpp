#include "Server.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

int main() {
    int rows = 3;
    int cols = 3;
    Server server(rows, cols);

    std::cout << "Server: Running. Press ENTER to exit...\n";

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        // Generate new random data
        Eigen::MatrixXf data = Eigen::MatrixXf::Random(rows, cols);

        // Write new data
        server.writeMemory(data);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Server: Write took " << duration.count() << "us\n";

        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Adjust the rate as per requirement
    }

    return 0;
}
