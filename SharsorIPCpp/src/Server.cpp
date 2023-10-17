#include <iostream>
#include <Eigen/Dense>
#include <cstring>
#include <typeinfo>
#include <ctime>

#include <MemUtils.hpp>
#include <SharsorIPCpp/Server.hpp>

namespace SharsorIPCpp {

    using LogType = Journal::LogType;

    // Define a type trait to check if a given type is a valid DType
    template <typename Scalar>
    struct IsValidDType {
        static constexpr bool value =
            std::is_same<Scalar, typename DTypeToCppType<DType::Float>::type>::value ||
            std::is_same<Scalar, typename DTypeToCppType<DType::Double>::type>::value ||
            std::is_same<Scalar, typename DTypeToCppType<DType::Int>::type>::value ||
            std::is_same<Scalar, typename DTypeToCppType<DType::Bool>::type>::value;
    };

    template <typename Scalar>
    Server<Scalar>::Server(int n_rows,
                   int n_cols,
                   std::string basename,
                   std::string name_space,
                   bool verbose,
                   VLevel vlevel,
                   bool force_reconnection)
        : n_rows(n_rows),
          n_cols(n_cols),
          _mem_config(basename, name_space),
          _verbose(verbose),
          _vlevel(vlevel),
          _force_reconnection(force_reconnection),
          _tensor_view(nullptr,
                       n_rows,
                       n_cols),
          _n_rows_view(nullptr,
                       1,
                       1),
          _n_cols_view(nullptr,
                       1,
                       1),
          _n_clients_view(nullptr,
                          1,
                          1),
          _dtype_view(nullptr,
                      1,
                      1),
          _journal(Journal(_getThisName()))
    {

        static_assert(IsValidDType<Scalar>::value, "Invalid data type provided.");

        if (_force_reconnection &&
                _verbose &&
                _vlevel > VLevel::V1)
        {
            std::string warn = std::string("Server will be initialized with force_reconnection to true. ") +
                    std::string("This can cause destructive behaviour if trying to run two servers concurrently on the ") +
                    std::string("same memory.");

            _journal.log(__FUNCTION__,
                 warn,
                 LogType::WARN);
        }

        if (_verbose &&
            _vlevel > VLevel::V1) {

            std::string info = std::string("Initializing Server at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::STAT);

        }

        _initSems(); // creates necessary semaphores

        _acquireSem(_mem_config.mem_path_data_sem,
                    _data_sem); // acquire shared data semaphore
        // Here to prevent access from any client (at this stage)

        _checkMem(_mem_config.mem_path,
                  _data_shm_fd); // checks if memory was already allocated

        MemUtils::initMem<Scalar>(n_rows,
                        n_cols,
                        _mem_config.mem_path,
                        _data_shm_fd,
                        _tensor_view,
                        _journal,
                        _verbose,
                        _vlevel); // initializes shared data

        _tensor_copy = Tensor<Scalar>::Zero(n_rows,
                                            n_cols); // used to hold
        // a copy of the shared tensor data

        _terminated = false; // just in case

    }

    template <typename Scalar>
    Server<Scalar>::~Server() {

        if (!_terminated) {

            close();
        }

    }

    template <typename Scalar>
    void Server<Scalar>::run()
    {

        if (!isRunning()) {

            _acquireSem(_mem_config.mem_path_server_sem,
                        _srvr_sem); // server semaphore

            _releaseSem(_mem_config.mem_path_data_sem,
                        _data_sem); // release data semaphore
            // so that clients can connect

            // set the running flag to true
            _running = true;

        }

    }

    template <typename Scalar>
    void Server<Scalar>::stop()
    {

        if (isRunning()) {

//            _acquireSem(_mem_config.mem_path_data_sem,
//                        _data_sem); // acquire data semaphore
            // clients won't be able to access the data

            _releaseSem(_mem_config.mem_path_server_sem,
                        _srvr_sem);

            _running = false;
        }

    }

    template <typename Scalar>
    bool Server<Scalar>::isRunning()
    {

        return _running;

    }

    template <typename Scalar>
    void Server<Scalar>::close()
    {

        stop(); // stop server if running

        _cleanUpAll(); // cleans up all memory,
        // semaphores included (if necessary)

        if (_verbose &&
            _vlevel > VLevel::V1) {

            std::string info = std::string("Closed server at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::STAT);

        }
    }

    template <typename Scalar>
    void Server<Scalar>::writeMemory(const Tensor<Scalar>& data) {

        if(data.rows() != n_rows || data.cols() != n_cols) {

            std::string error =
                    std::string("Data dimension mismatch. ") +
                    std::string("Expected a tensor of size ") +
                    std::to_string(n_rows) + std::string("x") + std::to_string(n_cols) +
                    std::string(", but provided tensor is ") +
                    std::to_string(data.rows()) + std::string("x") + std::to_string(data.cols());

            _journal.log(__FUNCTION__,
                         error,
                         LogType::EXCEP);

        }

        if (_running) {

            _tensor_view.block(0, 0, n_rows, n_cols) = data;

        }
        else {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the Run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }


    }

    template <typename Scalar>
    const MMap<Scalar>& Server<Scalar>::getTensorView() {

        if (!_running) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the Run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

        return _tensor_view;

    }

    template <typename Scalar>
    const Tensor<Scalar> &Server<Scalar>::getTensorCopy() {

        if (!_running) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the Run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

        _tensor_copy = _tensor_view;

        return _tensor_copy;

    }

    template <typename Scalar>
    void Server<Scalar>::_cleanUpAll()
    {

        if (!_terminated) {

            _cleanUpMem(_mem_config.mem_path,
                        _data_shm_fd); // closing shared

            _closeSems(); // closing semaphores

            if (_verbose &&
                _vlevel > VLevel::V1) {

                std::string info = std::string("Stopped server at ") +
                        _mem_config.mem_path;

                _journal.log(__FUNCTION__,
                     info,
                     LogType::STAT);

            }

            _terminated = true;

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_cleanUpMem(const std::string& mem_path,
                                    int& data_shm_fd)
    {
        // Unlinking from shared memory data
        shm_unlink(mem_path.c_str());

        if (_verbose && _vlevel > VLevel::V2) {

            std::string info = "Unlinked from memory at " + mem_path;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);
        }

        // Closing the file descriptor
        ::close(data_shm_fd);

        if (_verbose && _vlevel > VLevel::V2) {

            std::string info = "Closed file descriptor for " + mem_path;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);
        }
    }

    template <typename Scalar>
    void Server<Scalar>::_initSems()
    {

        _initSem(_mem_config.mem_path_server_sem,
                 _srvr_sem);

        _initSem(_mem_config.mem_path_data_sem,
                 _data_sem);
    }

    template <typename Scalar>
    void Server<Scalar>::_initSem(const std::string& sem_path,
                                  sem_t*& output_sem)
    {
        output_sem = sem_open(sem_path.c_str(),
                              O_CREAT, S_IRUSR | S_IWUSR,
                              1);

        if (output_sem == SEM_FAILED) {
            // Handle semaphore creation error

            std::string error = std::string("Failed to open semaphore");

            _journal.log(__FUNCTION__,
                error,
                LogType::EXCEP);
        }
        else {

            if (_verbose &&
                _vlevel > VLevel::V2) {

                std::string info = std::string("Opened semaphore at ") + sem_path;

                _journal.log(__FUNCTION__,
                    info,
                    LogType::INFO);
            }
        }
    }

    template <typename Scalar>
    void Server<Scalar>::_acquireSem(const std::string& sem_path,
                                     sem_t*& sem)
    {

        if (_verbose && _n_sem_acq_fail == 0 &&
                _vlevel > VLevel::V2) {

            std::string info = std::string("Acquiring server semaphore at ") +
                    sem_path;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);

        }

        // Acquire the semaphore
        if (_semWait(sem, 1.0) == -1) {

            _n_sem_acq_fail++;

            if (_n_sem_acq_fail > _n_acq_trials)
            { // we exceeded the number of allowed trials

                _journal.log(__FUNCTION__,
                             "Failed to acquire semaphore at",
                             LogType::EXCEP);

                _n_sem_acq_fail = 0; // reset counter

            }

            if (_verbose &&
                    _vlevel > VLevel::V0) {

                std::string warn = std::string("Semaphore acquisition at ") +
                        sem_path +
                        std::string(" failed. Trying to acquire it again...");

                _journal.log(__FUNCTION__,
                             warn,
                             LogType::WARN);

            }

            if (_force_reconnection) {

                _releaseSem(sem_path,
                            sem); // we try to release it, so that if a previous server
                // crashed, we now make the semaphore available for acquisition.
            }

            _acquireSem(sem_path,
                        sem); // recursive call. After _releaseSems(), this should now work

            if (_verbose && _n_sem_acq_fail > 0 &&
                    _vlevel > VLevel::V0) {

                std::string warn = std::string("Done.");

                _journal.log(__FUNCTION__,
                     warn,
                     LogType::WARN);

            }

        }

        if (_verbose &&
                _vlevel > VLevel::V2) {

            std::string info = std::string("Acquired server semaphore at ") +
                    sem_path;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);


        }

        _n_sem_acq_fail = 0; // reset counter

    }

    template <typename Scalar>
    void Server<Scalar>::_releaseSem(const std::string& sem_path,
                                     sem_t*& sem)
    {

        if (_verbose &&
                _vlevel > VLevel::V2) {

            std::string info = std::string("Releasing server semaphore at ") +
                        sem_path;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);
        }

        // Release the semaphore
        if (sem_post(sem) == -1) {

            // Handle semaphore release error

            _journal.log(__FUNCTION__,
                         "Failed to release semaphore at",
                         LogType::EXCEP);
        }

        if (_verbose &&
                _vlevel > VLevel::V2) {

            std::string info = std::string("Released server semaphore at ") +
                    sem_path;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_closeSems()
    {
        _closeSem(_mem_config.mem_path_server_sem, _srvr_sem);
        _closeSem(_mem_config.mem_path_data_sem, _data_sem);

    }

    template <typename Scalar>
    void Server<Scalar>::_closeSem(const std::string& sem_path,
                                   sem_t *&sem)
    {

        sem_close(sem); // close semaphore

        sem_unlink(sem_path.c_str()); // unlink semaphore

        if (_verbose &&
                _vlevel > VLevel::V2) {

            std::string info = std::string("Closed and unlinked server semaphore at ") +
                    sem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::INFO);

        }

    }

    template <typename Scalar>
    int Server<Scalar>::_semWait(sem_t* sem,
                                int timeout_seconds) {

        struct timespec timeout;

        clock_gettime(CLOCK_REALTIME, &timeout);

        timeout.tv_sec += timeout_seconds;

        while (true) {

            int result = sem_timedwait(sem, &timeout);

            if (result == 0) {
                // Successfully acquired the semaphore.
                return 0;
            } else if (result == -1 && errno == ETIMEDOUT) {
                // Timeout occurred.
                return -1;
            } else if (result == -1 && errno != EINTR) {
                // Other error occurred (excluding interrupt).
                return errno;
            }

        }
    }

    template <typename Scalar>
    void Server<Scalar>::_checkMem(const std::string& mem_path,
                                  int& data_shm_fd)
    {
        data_shm_fd = shm_open(mem_path.c_str(),
                               O_RDWR,
                               0);

        if (data_shm_fd != -1) {
            // Shared memory already exists, so we need to clean it up

            if (_verbose &&
                    _vlevel > VLevel::V0) {

                std::string warn = "Shared memory at " + mem_path +
                    " already exists. Clearing it up...";

                _journal.log(__FUNCTION__,
                             warn,
                             LogType::WARN);
            }

            _cleanUpMem(mem_path,
                        data_shm_fd);

            if (_verbose &&
                    _vlevel > VLevel::V0) {

                std::string warn = "Cleanup Done.";

                _journal.log(__FUNCTION__,
                             warn,
                             LogType::WARN);

            }
        }

        if (data_shm_fd != -1) {
            // Close the file descriptor opened for checking existence
            ::close(data_shm_fd);
        }
    }

    template <typename Scalar>
    std::string Server<Scalar>::_getThisName()
    {

        return _this_name;
    }

    // explicit instantiations for specific types
    template class Server<double>;
    template class Server<float>;
    template class Server<int>;
    template class Server<bool>;
}
