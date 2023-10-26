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
  SharedMemConfig mem_config("SharsorBool", "ConnectionTests");

  std::unique_ptr<DMMap<bool, MemLayoutDefault>> tensor_view_ptr;

  Tensor<bool, MemLayoutDefault> tensor_copy = Tensor<bool, MemLayoutDefault>::Zero(4,
                                   7);
  Tensor<bool, MemLayoutDefault> data2write(2, 5);
  data2write.setRandom();

  std::unique_ptr<DMMap<bool, MemLayoutDefault>> tensor_block_view = std::make_unique<DMMap<bool, MemLayoutDefault>>(
                                      helpers::createViewFrom<bool, MemLayoutDefault>(
                                                            tensor_copy,
                                                            1, 1,
                                                            4 - 2, 7 - 2));

  MemUtils::initMem<bool, MemLayoutDefault>(
                  4,
                  7,
                  mem_config.mem_path,
                  data_shm_fd,
                  tensor_view_ptr,
                  journal,
                  return_code,
                  true,
                  VLevel::V3);

  // write random block to mem
  MemUtils::write<bool, MemLayoutDefault>(data2write,
             *tensor_view_ptr,
             1, 1,
             journal,
             return_code,
             true,
             Journal::VLevel::V3);

  // only copy data to block view
  MemUtils::read<bool, SharsorIPCpp::MemLayoutDefault>(
                 0, 0,
                 *tensor_block_view,
                 *tensor_view_ptr,
                 journal,
                 return_code,
                 false,
                 VLevel::V3);

  std::cout << "Written block " << data2write << std::endl;
//  std::cout << "Block view " << *tensor_block_view << std::endl;
  std::cout << "Original Tensor " << tensor_copy << std::endl;

}

int main() {

    testMem();

}
