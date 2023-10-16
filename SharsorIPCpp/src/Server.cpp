#include "Server.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <cstring>
#include <typeinfo>
#include <ctime>

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
                   bool verbose)
        : n_rows(n_rows),
          n_cols(n_cols),
          _mem_config(basename, name_space),
          _verbose(verbose),
          _tensor_view(nullptr,
                       n_rows,
                       n_rows),
          _journal(Journal(_getThisName()))
    {

        _checkMem(); // checks if memory was already allocated

        static_assert(IsValidDType<Scalar>::value, "Invalid data type provided.");

        // Determine the size based on the Scalar type
        std::size_t data_size = sizeof(Scalar) * n_rows * n_cols;

        // Create shared memory
        _shm_fd = shm_open(_mem_config.mem_path.c_str(),
                           O_CREAT | O_RDWR,
                           S_IRUSR | S_IWUSR);

        if (_shm_fd == -1) {

            _journal.log(__FUNCTION__,
                         "Could not create shared memory.",
                         LogType::EXCEP);

        }

        // Set size
        if (ftruncate(_shm_fd, data_size) == -1) {

            _journal.log(__FUNCTION__,
                         "Could not set shared memory size.",
                         LogType::EXCEP);

        }

        if (_verbose) {

            std::string info = std::string("Opened shared memory at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::INFO);

        }

        // Map the shared memory
        Scalar* matrix_data = static_cast<Scalar*>(mmap(nullptr, data_size,
                                                        PROT_READ | PROT_WRITE,
                                                        MAP_SHARED, _shm_fd, 0));
        if (matrix_data == MAP_FAILED) {

            _journal.log(__FUNCTION__,
                         "Could not map memory size.",
                         LogType::EXCEP);

        }

        new (&_tensor_view) MMap<Scalar>(matrix_data, n_rows, n_cols);

        _tensor_copy = Tensor<Scalar>::Zero(n_rows, n_cols);

        if (_verbose) {

            std::string info = std::string("Mapped shared memory at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::INFO);

        }

        // Create necessary semaphores
        _initSems();

        _terminated = false;
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

            _acquireSems(); // acquire all necessary semaphores

            // set the running flag to true
            _running = true;

        }

    }

    template <typename Scalar>
    void Server<Scalar>::stop()
    {

        if (isRunning()) {

            _releaseSems();

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

        _cleanUpMem();

        if (_verbose) {

            std::string info = std::string("Closed Server at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::INFO);

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

            if (_verbose) {

                std::string error = std::string("Server is not running. ") +
                        std::string("Did you remember to call the Run() method?");

                _journal.log(__FUNCTION__,
                     error,
                     LogType::EXCEP);
            }
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

            _cleanUpMem(); // closing shared

            _closeSems(); // closing semaphores

            if (_verbose) {

                std::string info = std::string("Server stopped");

                _journal.log(__FUNCTION__,
                     info,
                     LogType::INFO);

            }

            _terminated = true;

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_cleanUpMem()
    {

        // unlinking from shared memory data
        shm_unlink(_mem_config.mem_path.c_str());

        if (_verbose) {

            std::string info = std::string("Unlinked from memory at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::INFO);

        }

        // closing file descriptor
        ::close(_shm_fd);

        if (_verbose) {

            std::string info = std::string("Closed file descriptor for ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::INFO);

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_initSems()
    {

        _srvr_sem = sem_open(_mem_config.mem_path_server_sem.c_str(),
                        O_CREAT,
                        S_IRUSR | S_IWUSR,
                        1);

        if (_srvr_sem == SEM_FAILED) {
            // Handle semaphore creation error

            std::string error =
                    std::string("Failed to open semaphore");

            _journal.log(__FUNCTION__,
                         error,
                         LogType::EXCEP);

        }
        else {

            if (_verbose) {

                std::string info = std::string("Opened semaphore at ") +
                        _mem_config.mem_path_server_sem;

                _journal.log(__FUNCTION__,
                     info,
                     LogType::INFO);

            }
        }
    }

    template <typename Scalar>
    void Server<Scalar>::_acquireSems()
    {

        if (_verbose) {

            std::string info = std::string("Acquiring server semaphore at ") +
                    _mem_config.mem_path_server_sem;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);

        }

        // Acquire the semaphore
        if (_semWait(_srvr_sem, 1.0) == -1) {

            // Handle semaphore acquisition error

            _journal.log(__FUNCTION__,
                         "Failed to acquire semaphore at",
                         LogType::EXCEP);

        }

        if (_verbose) {

            std::string info = std::string("Acquired server semaphore at ") +
                    _mem_config.mem_path_server_sem;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_releaseSems()
    {

        if (_verbose) {

            std::string info = std::string("Releasing server semaphore at ") +
                        _mem_config.mem_path_server_sem;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);
        }

        // Release the semaphore
        if (sem_post(_srvr_sem) == -1) {

            // Handle semaphore release error

            _journal.log(__FUNCTION__,
                         "Failed to release semaphore at",
                         LogType::EXCEP);
        }

        if (_verbose) {

            std::string info = std::string("Released server semaphore at ") +
                    _mem_config.mem_path_server_sem;

            _journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_closeSems()
    {

        sem_close(_srvr_sem); // close semaphore
        sem_unlink(_mem_config.mem_path_server_sem.c_str()); // unlink semaphore

        if (_verbose) {

            std::string info = std::string("Closed and unlinked server semaphore at ") +
                    _mem_config.mem_path_server_sem;

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
            // Handle EINTR (interrupted by a signal), and continue waiting.
        }
    }

    template <typename Scalar>
    void Server<Scalar>::_checkMem()
    {

        _shm_fd = shm_open(_mem_config.mem_path.c_str(), O_RDWR, 0);

        if (_shm_fd != -1) {
            // Shared memory already exists, so we need to clean it up

            std::string warn = std::string("Shared memory at ") +
                    _mem_config.mem_path + std::string(" already exists.\n")+
                    std::string("Clearning it up...");

            _journal.log(__FUNCTION__,
                 warn,
                 LogType::WARN);

            _cleanUpMem();

            _terminated = false;
        }

        if (_shm_fd == -1) {
            // Shared mem does not exist

            ::close(_shm_fd);// close the file descriptor
            // opened for checking existence
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
