#pragma once

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <csignal>

#include "Journal.hpp"
#include "DTypes.hpp"
#include "SharedMemConfig.hpp"

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
                   std::string basename = "MySharedMemory",
                   std::string name_space = "",
                   bool verbose = false);

            ~Server();

            void writeMemory(const Tensor<Scalar>& data);

            //  read only getter
            const MMap<Scalar>& getTensorView();

            // read only getter
            const Tensor<Scalar>& getTensorCopy();

            int n_rows;
            int n_cols;

            void Run();
            void Stop();
            void Close();

        private:

            bool _verbose = false;

            bool _terminated = false;

            bool _running = false;

            int _shm_fd; // shared memory file descriptor

            std::string _this_name = "SharsorIPCpp::Server";

            SharedMemConfig _mem_config;

            sem_t* _srvr_sem;

            Journal _journal;

            Tensor<Scalar> _tensor_copy;

            MMap<Scalar> _tensor_view;

            std::string GetThisName();

            void CleanUp();

            void CheckMem();

            void InitSem();

            void CloseSems();

            int semWait(sem_t* sem,
                        int timeout_seconds);

    };

}


