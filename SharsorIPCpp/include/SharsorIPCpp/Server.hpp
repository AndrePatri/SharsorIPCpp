#pragma once

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include "Journal.hpp"
#include "DTypes.hpp"

namespace SharsorIPCpp{

    template <typename Scalar>
    using Tensor = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    template <typename Scalar>
    using MMap = Eigen::Map<Tensor<Scalar>>;

    template <typename Scalar>
    class Server {

        public:

            Server(int n_rows,
                   int n_cols,
                   std::string memname = "MySharedMemory",
                   std::string name_space = "");

            ~Server();

            void writeMemory(const Tensor<Scalar>& data);

            //  read only getter
            const MMap<Scalar>& getTensorView();

            // read only getter
            const Tensor<Scalar>& getTensorCopy();

            int n_rows;
            int n_cols;

        private:

            int _shm_fd; // shared memory file descriptor

            std::string _shared_mem_name;
            std::string _namespace;

            Journal _journal;

            Tensor<Scalar> _tensor_copy;

            MMap<Scalar> _tensor_view;

            std::string GetThisName();

    };

}


