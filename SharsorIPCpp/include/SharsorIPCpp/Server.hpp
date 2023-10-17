#pragma once

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <csignal>
#include <memory>

#include <SharedMemConfig.hpp>

#include <MemUtils.hpp>

#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

namespace SharsorIPCpp{

    template <typename Scalar>
    class Server {

        public:

            typedef std::weak_ptr<Server> WeakPtr;
            typedef std::shared_ptr<Server> Ptr;
            typedef std::unique_ptr<Server> UniquePtr;

            Server(int n_rows,
                   int n_cols,
                   std::string basename = "MySharedMemory",
                   std::string name_space = "",
                   bool verbose = false,
                   VLevel vlevel = VLevel::V0,
                   bool force_reconnection = false);

            ~Server();

            void writeMemory(const Tensor<Scalar>& data);

            //  read only getter
            const MMap<Scalar>& getTensorView();

            // read only getter
            const Tensor<Scalar>& getTensorCopy();

            int n_rows;
            int n_cols;

            void run();
            void stop();
            void close();

            bool isRunning();

        private:

            bool _verbose = false;

            bool _terminated = false;

            bool _running = false;

            bool _force_reconnection = false;

            int _data_shm_fd; // shared memory file descriptor
            int _nrows_shm_fd,
                _ncols_shm_fd,
                _n_clients_shm_fd,
                _dtype_shm_fd;

            int _n_sem_acq_fail = 0;
            int _n_acq_trials = 10;

            std::string _this_name = "SharsorIPCpp::Server";

            VLevel _vlevel = VLevel::V0; // minimal debug info

            SharedMemConfig _mem_config;

            sem_t* _srvr_sem; // semaphore for servers uniqueness
            sem_t* _data_sem; // semaphore for safe data access

            Journal _journal; // for rt-friendly logging

            Tensor<Scalar> _tensor_copy; // copy (not view) of the tensor

            MMap<Scalar> _tensor_view; // view of the tensor
            // auxiliary views
            MMap<int> _n_rows_view,
                      _n_cols_view,
                      _n_clients_view,
                      _dtype_view;

            std::string _getThisName();

            void _initMems();

            void _initSems();

            void _closeSems();

            void _cleanUpAll();

    };

}


