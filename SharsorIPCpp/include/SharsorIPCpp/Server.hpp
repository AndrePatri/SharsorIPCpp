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

#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

namespace SharsorIPCpp{

    template <typename Scalar>
    using Tensor = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    template <typename Scalar>
    using MMap = Eigen::Map<Tensor<Scalar>>;

    using VLevel = Journal::VLevel;

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

            int _shm_fd; // shared memory file descriptor

            int _n_sem_acq_fail = 0;
            int _n_acq_trials = 10;

            std::string _this_name = "SharsorIPCpp::Server";

            VLevel _vlevel = VLevel::V0; // minimal debug info

            SharedMemConfig _mem_config;

            sem_t* _srvr_sem;

            Journal _journal;

            Tensor<Scalar> _tensor_copy;

            MMap<Scalar> _tensor_view;

            std::string _getThisName();

            void _checkMem();

            void _initMem();

            void _cleanUpMem();

            void _initSems();

            void _acquireSems();

            void _releaseSems();

            void _closeSems();

            void _cleanUpAll();

            int _semWait(sem_t* sem,
                        int timeout_seconds);

    };

}


