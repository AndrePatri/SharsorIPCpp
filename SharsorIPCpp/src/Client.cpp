#include <iostream>
#include <Eigen/Dense>
#include <cstring>
#include <typeinfo>
#include <ctime>

#include <MemUtils.hpp>
#include <SharsorIPCpp/Client.hpp>

namespace SharsorIPCpp {

    template <typename Scalar>
    Client<Scalar>::Client(
                   std::string basename,
                   std::string name_space,
                   bool verbose,
                   VLevel vlevel)
        : _mem_config(basename, name_space),
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
          _isrunning_view(nullptr,
                          1,
                          1),
          _journal(Journal(_getThisName()))
    {

        static_assert(MemUtils::IsValidDType<Scalar>::value,
                "Invalid data type provided.");


        _terminated = false; // just in case

    }

    template <typename Scalar>
    Client<Scalar>::~Client() {

        if (!_terminated) {

            close();
        }

    }

    template <typename Scalar>
    void Client<Scalar>::attach()
    {
        if (_verbose &&
            _vlevel > VLevel::V1) {

            std::string info = std::string("Initializing Client at ") +
                    _mem_config.mem_path;

            _journal.log(__FUNCTION__,
                 info,
                 LogType::STAT);

        }

        _initMetaMem(); // initializes meta-memory

        _waitForServer();  // waits until server is properly initialized

        _initSems(); // creates necessary semaphores

        // acquire shared data semaphore or waits for it
        // (at this point is actually guaranteed to be free anyway)
        _acquireData();

        _checkDType(); // checks data type consistency

        n_rows = _n_rows_view(0, 0);
        n_cols = _n_cols_view(0, 0);
        _n_clients_view(0, 0) = _n_clients_view(0, 0) + 1; // increase clients counter

        // we have now all the info to create the shared tensor
        MemUtils::initMem<Scalar>(n_rows,
                        n_cols,
                        _mem_config.mem_path,
                        _data_shm_fd,
                        _tensor_view,
                        _journal,
                        _verbose,
                        _vlevel);

        _tensor_copy = Tensor<Scalar>::Zero(n_rows,
                                            n_cols); // used to hold
        // a copy of the shared tensor data

        // releasing data semaphore so that other clients/the server can access the tensor
        _releaseData();

        _attached = true;
    }

    template <typename Scalar>
    void Client<Scalar>::detach()
    {
        if (_attached) {

            _acquireData();

            _n_clients_view(0, 0) = _n_clients_view(0, 0) - 1; // increase clients counter

            // releasing data semaphore so that other clients/the server can access the tensor
            _releaseData();

            _attached = false;

        }
    }

    template <typename Scalar>
    bool Client<Scalar>::isAttached()
    {

        return _attached;

    }

    template <typename Scalar>
    void Client<Scalar>::close()
    {

        detach(); // detach from server if not attached

        _cleanMems(); // cleans up all memory,
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
    void Client<Scalar>::writeTensor(const Tensor<Scalar>& data,
                                     int row,
                                     int col) {

        if (_attached) {

            _acquireData();

            MemUtils::write(data,
                            _tensor_view,
                            row, col,
                            _journal,
                            _verbose,
                            _vlevel);

            _releaseData();

        }

        if (!_attached && _verbose) {

            std::string error = std::string("Client is not registered to the Server. ") +
                    std::string("Did you remember to call the attach() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

    }

    template <typename Scalar>
    void Client<Scalar>::readTensor(Tensor<Scalar>& output,
                                    int row, int col) {

        if (_attached) {

            _acquireData();

            MemUtils::read(row, col,
                           output,
                           _tensor_view,
                           _journal,
                           _verbose,
                           _vlevel);

            _releaseData();

        }

        if (!_attached && _verbose) {

            std::string error = std::string("Client is not registered to the Server. ") +
                    std::string("Did you remember to call the attach() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

    }

//    template <typename Scalar>
//    const MMap<Scalar>& Client<Scalar>::getTensorView() {

//        // note: this is dangerous since it allows modification to the data without
//        // any synchronization, but can be useful to interface with other libraries (i.e. Torch, Numpy)

//        if (!_attached && _verbose) {

//            std::string error = std::string("Client is not registered to the Server. ") +
//                    std::string("Did you remember to call attach attach() method?");

//            _journal.log(__FUNCTION__,
//                 error,
//                 LogType::EXCEP);

//        }

//        return _tensor_view;

//    }

    template <typename Scalar>
    void Client<Scalar>::_waitForServer()
    {
        while(!(_isrunning_view(0, 0) > 0)) {

            if (_verbose &&
                _vlevel > VLevel::V0) {

                std::string info = std::string("Waiting transition of Server at ") +
                        _mem_config.mem_path +
                        std::string(" to running state...");

                _journal.log(__FUNCTION__,
                     info,
                     LogType::WARN);

            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        }
    }

    template <typename Scalar>
    void Client<Scalar>::_checkDType()
    {
        if (_dtype_view(0, 0) != sizeof(Scalar)) {

            // not impeccable: different types may have in general different sizes

            if (_verbose) {

                std::string error = std::string("Client initialized with element size of ") +
                        std::to_string(sizeof(Scalar)) +
                        std::string(", while the Server was initialized with size ") +
                        std::to_string(_dtype_view(0, 0));

                _journal.log(__FUNCTION__,
                             error,
                             LogType::EXCEP);
            }

        }
    }

    template <typename Scalar>
    void Client<Scalar>::_acquireData()
    {

        MemUtils::acquireSem<Scalar>(_mem_config.mem_path_data_sem,
                                     _data_sem,
                                     _n_acq_trials,
                                     _n_sem_acq_fail,
                                     _journal,
                                     false, // no verbosity (this is called very frequently)
                                     _verbose,
                                     _vlevel);

    }

    template <typename Scalar>
    void Client<Scalar>::_releaseData()
    {

        MemUtils::releaseSem<Scalar>(_mem_config.mem_path_data_sem,
                             _data_sem,
                             _journal,
                             false, // no verbosity (this is called very frequently)
                             _vlevel);

    }

    template <typename Scalar>
    void Client<Scalar>::_cleanMetaMem()
    {
        // closing file descriptors and but not unlinking
        // memory

        MemUtils::cleanUpMem<Scalar>(_mem_config.mem_path_nrows,
                             _nrows_shm_fd,
                             _journal,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem<Scalar>(_mem_config.mem_path_ncols,
                             _ncols_shm_fd,
                             _journal,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem<Scalar>(_mem_config.mem_path_clients_counter,
                             _n_clients_shm_fd,
                             _journal,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem<Scalar>(_mem_config.mem_path_dtype,
                             _dtype_shm_fd,
                             _journal,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem<Scalar>(_mem_config.mem_path_isrunning,
                             _isrunning_shm_fd,
                             _journal,
                             _verbose,
                             _vlevel,
                             _unlink_data);

    }

    template <typename Scalar>
    void Client<Scalar>::_cleanMems()
    {

        if (!_terminated) {

            MemUtils::cleanUpMem<Scalar>(_mem_config.mem_path,
                                 _data_shm_fd,
                                 _journal,
                                 _verbose,
                                 _vlevel,
                                 false); // closing but no unlinking

            _cleanMetaMem(); // closes, but doesn't unlink, aux. data

            _closeSems(); // closing semaphores

            if (_verbose &&
                _vlevel > VLevel::V1) {

                std::string info = std::string("Closed client at ") +
                        _mem_config.mem_path;

                _journal.log(__FUNCTION__,
                     info,
                     LogType::STAT);

            }

            _terminated = true;

        }

    }

    template <typename Scalar>
    void Client<Scalar>::_initMetaMem()
    {

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

        MemUtils::initMem<bool>(1,
                        1,
                        _mem_config.mem_path_isrunning,
                        _isrunning_shm_fd,
                        _isrunning_view,
                        _journal,
                        _verbose,
                        _vlevel);

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
        // closes semaphores but doesn't unlink them
        MemUtils::closeSem<Scalar>(_mem_config.mem_path_data_sem,
                                   _data_sem,
                                   _journal,
                                   _verbose,
                                   _vlevel,
                                   false);

    }

    template <typename Scalar>
    std::string Client<Scalar>::_getThisName()
    {

        return _this_name;
    }

    // explicit instantiations for specific supported types
    template class Client<double>;
    template class Client<float>;
    template class Client<int>;
    template class Client<bool>;
}

