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

        MemUtils::acquireSem<Scalar>(_mem_config.mem_path_data_sem,
                                     _data_sem,
                                     _n_acq_trials,
                                     _n_sem_acq_fail,
                                     _journal,
                                     _force_reconnection,
                                     _verbose,
                                     _vlevel); // acquire shared data semaphore
        // Here to prevent access from any client (at this stage)


        MemUtils::checkMem<Scalar>(_mem_config.mem_path,
                                _data_shm_fd,
                                _journal,
                                _verbose,
                                _vlevel); // checks if memory was already allocated

        _initMem();

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

            MemUtils::acquireSem<Scalar>(_mem_config.mem_path_server_sem,
                                         _srvr_sem,
                                         _n_acq_trials,
                                         _n_sem_acq_fail,
                                         _journal,
                                         _force_reconnection,
                                         _verbose,
                                         _vlevel);

            MemUtils::releaseSem<Scalar>(_mem_config.mem_path_data_sem,
                                 _data_sem,
                                 _journal,
                                 _verbose,
                                 _vlevel); // release data semaphore
                    // so that clients can connect

            // set the running flag to true
            _running = true;

        }

    }

    template <typename Scalar>
    void Server<Scalar>::stop()
    {

        if (isRunning()) {


            MemUtils::releaseSem<Scalar>(_mem_config.mem_path_server_sem,
                        _srvr_sem,
                        _journal,
                        _verbose,
                        _vlevel);

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

        if (_running) {

            MemUtils::write(data,
                            _tensor_view,
                            _journal);

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

            MemUtils::cleanUpMem<Scalar>(_mem_config.mem_path,
                                 _data_shm_fd,
                                 _journal,
                                 _verbose,
                                 _vlevel);

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
    void Server<Scalar>::_initMems()
    {

        MemUtils::initMem<Scalar>(n_rows,
                        n_cols,
                        _mem_config.mem_path,
                        _data_shm_fd,
                        _tensor_view,
                        _journal,
                        _verbose,
                        _vlevel); // initializes shared data
    }

    template <typename Scalar>
    void Server<Scalar>::_initSems()
    {

        MemUtils::initSem<Scalar>(_mem_config.mem_path_server_sem,
                                  _srvr_sem,
                                  _journal,
                                  _verbose,
                                  _vlevel);

        MemUtils::initSem<Scalar>(_mem_config.mem_path_data_sem,
                                  _data_sem,
                                  _journal,
                                  _verbose,
                                  _vlevel);

    }

    template <typename Scalar>
    void Server<Scalar>::_closeSems()
    {
        MemUtils::closeSem<Scalar>(_mem_config.mem_path_server_sem,
                                   _srvr_sem,
                                   _journal,
                                   _verbose,
                                   _vlevel);

        MemUtils::closeSem<Scalar>(_mem_config.mem_path_data_sem,
                                   _data_sem,
                                   _journal,
                                   _verbose,
                                   _vlevel);

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
