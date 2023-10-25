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
#include <SharsorIPCpp/ReturnCodes.hpp>

namespace SharsorIPCpp{

    template <typename Scalar,
              int Layout = Eigen::ColMajor>
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

            bool writeTensor(const Tensor<Scalar, Layout>& data,
                             int row = 0,
                             int col = 0);

            bool readTensor(Tensor<Scalar, Layout>& output,
                            int row = 0, int col = 0); // copies
            // underlying shared tensor data to the output

            bool readTensor(MMap<Scalar, Layout>& output,
                            int row = 0, int col = 0); // copies
            // underlying shared tensor data to the output Map

            void run();
            void stop();
            void close();

            bool isRunning();

            int getNClients();

            int getNRows();
            int getNCols();

            DType getScalarType() const;

        private:

            bool _unlink_data = true; // will also unlink data
            // when freeing shared memory

            bool _verbose = false;

            bool _terminated = false;

            bool _running = false;

            bool _force_reconnection = false;

            int _n_rows = -1;
            int _n_cols = -1;

            int _n_clients = -1;

            int _data_shm_fd; // shared memory file descriptor
            int _nrows_shm_fd,
                _ncols_shm_fd,
                _n_clients_shm_fd,
                _dtype_shm_fd,
                _isrunning_shm_fd,
                _mem_layout_shm_fd;

            static const int _mem_layout = Layout;

            int _n_sem_acq_fail = 0;
            int _n_acq_trials = 10;       

            std::string _this_name = "SharsorIPCpp::Server";

            VLevel _vlevel = VLevel::V0; // minimal debug info

            SharedMemConfig _mem_config;

            sem_t* _srvr_sem; // semaphore for servers uniqueness
            sem_t* _data_sem; // semaphore for safe data access

            Journal _journal; // for rt-friendly logging

            ReturnCode _return_code; // overwritten by all methods
            // this is to avoid dyn. allocation

            Tensor<Scalar, Layout> _tensor_copy; // copy (not view) of the tensor

            MMap<Scalar, Layout> _tensor_view; // view of the tensor
            // auxiliary views
            MMap<int, Layout> _n_rows_view,
                      _n_cols_view,
                      _n_clients_view,
                      _dtype_view,
                      _mem_layout_view;
            MMap<bool, Layout> _isrunning_view;

            std::string _getThisName();

            void _initDataMem();
            void _initMetaMem();

            void _initSems();

            void _acquireSemWait(const std::string& sem_path,
                             sem_t*& sem);
            bool _acquireSemRt(const std::string& sem_path,
                             sem_t*& sem);
            void _releaseSem(const std::string& sem_path,
                             sem_t*& sem);

            bool _acquireData(bool blocking = false);
            void _releaseData();

            void _closeSems();

            void _cleanMetaMem();
            void _cleanMems();

    };

}


