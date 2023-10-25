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

void testMem() {

  using namespace SharsorIPCpp;

  ReturnCode return_code = ReturnCode::RESET;
  Journal journal = Journal("aaa");

  int data_shm_fd; // shared memory file descriptor
  SharedMemConfig mem_config("SharsorInt", "ConnectionTests");

  MMap<int, RowMajor> tensor_view(nullptr,
                                  -1,
                                  -1); // view of the tensor

  Tensor<int, RowMajor> tensor_copy = Tensor<int, RowMajor>::Zero(4,
                                   7);

  std::unique_ptr<DMMap<int, RowMajor>> tensor_block_view = std::make_unique<DMMap<int, RowMajor>>(
                                      helpers::createViewFrom<int, RowMajor>(
                                                            tensor_copy,
                                                            1, 1,
                                                            4 - 2, 7 - 2));

  MemUtils::initMem<int, SharsorIPCpp::RowMajor>(
                  4,
                  7,
                  mem_config.mem_path,
                  data_shm_fd,
                  tensor_view,
                  journal,
                  return_code,
                  true,
                  VLevel::V3);

  MemUtils::read<int, SharsorIPCpp::RowMajor>(
                 1, 1,
                 *tensor_block_view,
                 tensor_view,
                 journal,
                 return_code,
                 false,
                 VLevel::V3);

}


int main() {

    testMem();

}
