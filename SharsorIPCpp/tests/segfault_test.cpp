#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <csignal>
#include <thread>
#include <chrono>
#include <numeric>
#include <vector>
#include <string>

#include <MemUtils.hpp>
#include <SharsorIPCpp/DTypes.hpp>
#include <SharsorIPCpp/Helpers.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharedMemConfig.hpp>

using namespace SharsorIPCpp;

ReturnCode return_code = ReturnCode::RESET;
Journal journal = Journal("aaa");

int _data_shm_fd; // shared memory file descriptor
SharedMemConfig mem_config;

MMap<int, RowMajor> tensor_view(nullptr,
                                -1,
                                -1); // view of the tensor

std::unique_ptr<DMMap<int, RowMajor>> tensor_block_view;
tensor_block_view = std::make_unique<DMMap<int, RowMajor>>(
                                    helpers::createViewFrom<int, RowMajor>(
                                                          tensor_copy,
                                                          1, 1,
                                                          rows - 2, cols - 2));

MemUtils::initMem<Scalar, Layout>(4,
                7,
                mem_config.mem_path,
                data_shm_fd,
                tensor_view,
                journal,
                return_code,
                true,
                VLevel::V3);

output = tensor_view.block(row, col,
                           output.rows(),
                           output.cols()).eval();
