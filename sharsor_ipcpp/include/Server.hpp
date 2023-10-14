#pragma once

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

namespace SharsorIPCpp{

  using Tensor = Eigen::MatrixXf;
  using MMap = Eigen::Map<Tensor>;

  class Server {

    public:
        Server(int rows, int cols, std::string memname = "MySharedMemory");
        ~Server();

        void writeMemory(const Tensor& data);

        //  read only getter
        const MMap& getTensorView();

        // read only getter
        const Tensor& getTensorCopy();

    private:
        int rows_;
        int cols_;

        MMap tensor_view_;

        Tensor tensor_copy_;

        std::string _shared_mem_name;
        int shm_fd_; // shared memory file descriptor
  };

}


