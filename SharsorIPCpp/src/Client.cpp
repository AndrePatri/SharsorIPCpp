#include <iostream>
#include <Eigen/Dense>
#include <cstring>
#include <typeinfo>
#include <ctime>

#include <MemUtils.hpp>
#include <SharsorIPCpp/Client.hpp>

namespace SharsorIPCpp {

    template <typename Scalar>
    Client<Scalar>::Client(int n_rows,
                   int n_cols,
                   std::string basename,
                   std::string name_space,
                   bool verbose,
                   VLevel vlevel)
        : n_rows(n_rows),
          n_cols(n_cols),
          _mem_config(basename, name_space),
          _verbose(verbose),
          _vlevel(vlevel),
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

        static_assert(MemUtils::IsValidDType<Scalar>::value, "Invalid data type provided.");

        if (_verbose &&
            _vlevel > VLevel::V1) {

            std::string info = std::string("Initializing Client at ") +
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
                                     false,
                                     _verbose,
                                     _vlevel); // acquire shared data semaphore
        // Here to prevent access from any client (at this stage)


        MemUtils::checkMem<Scalar>(_mem_config.mem_path,
                                _data_shm_fd,
                                _journal,
                                _verbose,
                                _vlevel); // checks if memory was already allocated

        _initMems();

        _tensor_copy = Tensor<Scalar>::Zero(n_rows,
                                            n_cols); // used to hold
        // a copy of the shared tensor data

        _terminated = false; // just in case

    }

    template <typename Scalar>
    Client<Scalar>::~Client() {

        if (!_terminated) {

            close();
        }

    }

    template <typename Scalar>
    bool Client<Scalar>::isRunning()
    {

        return _running;

    }

    template <typename Scalar>
    void Client<Scalar>::close()
    {

        _cleanUpAll(); // cleans up all memory,
        // semaphores included (if necessary)

        if (_verbose &&
            _vlevel > VLevel::V1) {

            std::string info = std::string("Closed client at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::STAT);

        }
    }

    template <typename Scalar>
    void Client<Scalar>::writeMemory(const Tensor<Scalar>& data) {

        if (_running) {

            MemUtils::write(data,
                            _tensor_view,
                            _journal);

        }
        else {

            std::string error = std::string("Client is not running. ") +
                    std::string("Did you remember to call the Run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

    }

    template <typename Scalar>
    const MMap<Scalar>& Client<Scalar>::getTensorView() {

        if (!_running) {

            std::string error = std::string("Client is not running. ") +
                    std::string("Did you remember to call the Run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

        return _tensor_view;

    }

    template <typename Scalar>
    const Tensor<Scalar> &Client<Scalar>::getTensorCopy() {

        if (!_running) {

            std::string error = std::string("Client is not running. ") +
                    std::string("Did you remember to call the Run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

        _tensor_copy = _tensor_view;

        return _tensor_copy;

    }

    template <typename Scalar>
    void Client<Scalar>::_cleanUpAll()
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

                std::string info = std::string("Stopped client at ") +
                        _mem_config.mem_path;

                _journal.log(__FUNCTION__,
                     info,
                     LogType::STAT);

            }

            _terminated = true;

        }

    }

    template <typename Scalar>
    void Client<Scalar>::_initMems()
    {

        MemUtils::initMem<Scalar>(n_rows,
                        n_cols,
                        _mem_config.mem_path,
                        _data_shm_fd,
                        _tensor_view,
                        _journal,
                        _verbose,
                        _vlevel); // initializes shared data

        // auxiliary data
        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_nrows,
                        _nrows_shm_fd,
                        _n_rows_view,
                        _journal,
                        _verbose,
                        _vlevel);

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_ncols,
                        _ncols_shm_fd,
                        _n_cols_view,
                        _journal,
                        _verbose,
                        _vlevel);

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_clients_counter,
                        _n_clients_shm_fd,
                        _n_clients_view,
                        _journal,
                        _verbose,
                        _vlevel);

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_dtype,
                        _dtype_shm_fd,
                        _dtype_view,
                        _journal,
                        _verbose,
                        _vlevel);

        _n_rows_view(0, 0) = n_rows;
        _n_cols_view(0, 0) = n_cols;
        _n_clients_view(0, 0) = 0;
        _dtype_view(0, 0) = sizeof(Scalar);

    }

    template <typename Scalar>
    void Client<Scalar>::_initSems()
    {

        MemUtils::initSem<Scalar>(_mem_config.mem_path_data_sem,
                                  _data_sem,
                                  _journal,
                                  _verbose,
                                  _vlevel);

    }

    template <typename Scalar>
    void Client<Scalar>::_closeSems()
    {

        MemUtils::closeSem<Scalar>(_mem_config.mem_path_data_sem,
                                   _data_sem,
                                   _journal,
                                   _verbose,
                                   _vlevel);

    }

    template <typename Scalar>
    std::string Client<Scalar>::_getThisName()
    {

        return _this_name;
    }

    // explicit instantiations for specific types
    template class Client<double>;
    template class Client<float>;
    template class Client<int>;
    template class Client<bool>;
}

