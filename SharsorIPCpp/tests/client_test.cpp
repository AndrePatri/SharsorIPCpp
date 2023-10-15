#include "Client.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    int rows = 3;
    int cols = 3;
    Client client(rows, cols);

    while(true) {
        auto start = std::chrono::high_resolution_clock::now();

        // Read memory
        client.readMemory();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Client: Read took " << duration.count() << "us\n";

        // Sleep for a bit before reading again
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Adjust as per requirement
    }

    return 0;
}
