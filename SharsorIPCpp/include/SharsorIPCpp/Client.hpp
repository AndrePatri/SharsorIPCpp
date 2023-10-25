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
#include <thread>
#include <chrono>

#include <SharedMemConfig.hpp>

#include <MemUtils.hpp>

#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

namespace SharsorIPCpp{

    template <typename Scalar,
              int Layout = MemLayoutDefault>

    class Client {

        public:

            typedef std::weak_ptr<Client> WeakPtr;
            typedef std::shared_ptr<Client> Ptr;
            typedef std::unique_ptr<Client> UniquePtr;

            Client(std::string basename = "MySharedMemory",
                   std::string name_space = "",
                   bool verbose = false,
                   VLevel vlevel = VLevel::V0);

            ~Client();

            bool writeTensor(const Tensor<Scalar, Layout>& data,
                             int row = 0,
                             int col = 0);

            bool readTensor(Tensor<Scalar, Layout>& output,
                            int row = 0, int col = 0); // copies
            // underlying shared tensor data to the output

            bool readTensor(DMMap<Scalar, Layout>& output,
                            int row = 0, int col = 0); // copies
            // underlying shared tensor data to the output Map

            void attach();
            void detach();

            void close();

            bool isAttached();

            int getNRows();
            int getNCols();

            DType getScalarType() const;

            int getMemLayout() const;

        private:

            bool _unlink_data = false; // will never unlink data
            // when cleaning shared memory

            bool _verbose = false;

            bool _terminated = false;

            bool _attached = false;

            int _n_rows = -1;
            int _n_cols = -1;

            int _data_shm_fd; // shared memory file descriptor
            int _nrows_shm_fd,
                _ncols_shm_fd,
                _n_clients_shm_fd,
                _dtype_shm_fd,
                _isrunning_shm_fd,
                _mem_layout_shm_fd;

            static const int _mem_layout = Layout;

            int _n_sem_acq_fail = 0;
            int _n_acq_trials = 100;

            std::string _this_name = "SharsorIPCpp::Client";

            VLevel _vlevel = VLevel::V0; // minimal debug info

            Scalar _scalarType; // scalar type of this class

            SharedMemConfig _mem_config;

            sem_t* _data_sem; // semaphore for safe data access

            ReturnCode _return_code; // overwritten by all methods
            // this is to avoid dyn. allocation

            Journal _journal; // for rt-friendly logging

            Tensor<Scalar, Layout> _tensor_copy; // copy (not view) of the tensor

            MMap<Scalar, Layout> _tensor_view; // view of the tensor
            // auxiliary views
            MMap<int, Layout> _n_rows_view,
                      _n_cols_view,
                      _n_clients_view,
                      _dtype_view,
                      _mem_layout_view;
            MMap<bool, Layout> _isrunning_view;

            void _acquireSemWait(const std::string& sem_path,
                             sem_t*& sem);
            bool _acquireSemRt(const std::string& sem_path,
                             sem_t*& sem);
            void _releaseSem(const std::string& sem_path,
                             sem_t*& sem);

            bool _acquireData(bool blocking = false);
            void _releaseData();

            void _waitForServer();

            std::string _getThisName();

            void _checkDType();
            void _checkMemLayout(); // checks if mem. layout is
            // consistent with Server

            void _initDataMem();
            void _initMetaMem();

            void _initSems();

            void _closeSems();

            void _cleanMetaMem();
            void _cleanMems();

    };

}
