// Copyright (C) 2023  Andrea Patrizi (AndrePatri)
// 
// This file is part of SharsorIPCpp and distributed under the General Public License version 2 license.
// 
// SharsorIPCpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// SharsorIPCpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with SharsorIPCpp.  If not, see <http://www.gnu.org/licenses/>.
// 
#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <csignal>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Helpers.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <test_utils.hpp>

using namespace SharsorIPCpp;

using VLevel = Journal::VLevel;

static std::string name_space = "MemTests";

static Journal journal("MemTests");

int N_ITERATIONS = 10000;

#include <sys/resource.h>

double getMemoryUsageGB() {
    rusage usage;
    getrusage(RUSAGE_SELF, &usage);

    // Memory usage is in kilobytes on Linux
    double usedKB = static_cast<double>(usage.ru_maxrss);
    double usedGB = usedKB / (1024 * 1024);  // Convert kilobytes to gigabytes
    return usedGB;
}

// Define a structure to hold both the scalar type and the memory layout
template<typename T, int Layout>
struct TypeWithLayout {
    using type = T;
    static const int layout = Layout;
};

// List of types to test against
using MyTypes = ::testing::Types<
    TypeWithLayout<double, RowMajor>
>;

// normal tensor API
template <typename P>
class MemTest : public ::testing::Test {
protected:

    using ScalarType = typename P::type;
    static const int layout = P::layout;

    MemTest() : rows(1000),
                   cols(1000),
                   iterations(N_ITERATIONS),
                   server_ptr(new Server<ScalarType, layout>(rows, cols,
                                     "SharsorIPCpp",
                                     name_space,
                                     true,
                                     VLevel::V3,
                                     true)),
                   tensor_copy(rows, cols) {
        server_ptr->run();
    }

    void SetUp() override {
        // Initialization code (if needed)
    }

    void TearDown() override {
        server_ptr->close();
        // Cleanup code (if needed)
    }

    int rows;
    int cols;
    int iterations;
    typename Server<ScalarType, layout>::UniquePtr server_ptr;
    Tensor<ScalarType, layout> tensor_copy;

};

// normal tensor API
TYPED_TEST_SUITE_P(MemTest);

TYPED_TEST_P(MemTest, TestRAMAllocation) {

    using ScalarType = typename TestFixture::ScalarType;
    const int layout = TestFixture::layout;

    check_comp_type(journal);

    journal.log("MemTest", "\nBenchmarking memory...\n",
                Journal::LogType::STAT);
        
    double ram = 0.0;
    std::vector<double> memoryUsageHistory;
    memoryUsageHistory.reserve(this->iterations);

    for (int i = 0; i < this->iterations; ++i) {

        Tensor<ScalarType, layout> myData(this->rows, this->cols);
        myData.setRandom(); // we generate a random tensor of the right size

        // we measure the time to write it on the memory
        auto startWrite = std::chrono::high_resolution_clock::now();
        this->server_ptr->write(myData);

        memoryUsageHistory.push_back(getMemoryUsageGB());

    }
    
    // Optionally, you can access historical memory usage values in the vector (memoryUsageHistory)
    for (const auto& usage : memoryUsageHistory) {
        std::cout << "Memory usage: " << usage << " GB" << std::endl;
    }

}

// Register the tests
REGISTER_TYPED_TEST_SUITE_P(MemTest, TestRAMAllocation);
INSTANTIATE_TYPED_TEST_SUITE_P(MemTests, MemTest, MyTypes);

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

