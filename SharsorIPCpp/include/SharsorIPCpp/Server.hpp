#pragma once

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include "Journal.hpp"

namespace SharsorIPCpp{

  using Tensor = Eigen::MatrixXf;
  using MMap = Eigen::Map<Tensor>;

  class Server {

    public:

        Server(int n_rows,
               int n_cols,
               std::string memname = "MySharedMemory",
               std::string name_space = "");

        ~Server();

        void writeMemory(const Tensor& data);

        //  read only getter
        const MMap& getTensorView();

        // read only getter
        const Tensor& getTensorCopy();

        int n_rows;
        int n_cols;

    private:

        int _shm_fd; // shared memory file descriptor

        std::string _shared_mem_name;
        std::string _namespace;

        Journal _journal;

        Tensor _tensor_copy;

        MMap _tensor_view;

        std::string GetThisName();

  };

}


