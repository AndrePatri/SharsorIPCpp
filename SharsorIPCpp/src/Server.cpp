#include <iostream>
#include <Eigen/Dense>
#include <cstring>
#include <typeinfo>
#include <ctime>

#include <MemUtils.hpp>
#include <SharsorIPCpp/Server.hpp>

namespace SharsorIPCpp {

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
          _isrunning_view(nullptr,
                      1,
                      1),
          _journal(Journal(_getThisName()))
    {

        static_assert(MemUtils::IsValidDType<Scalar>::value, "Invalid data type provided.");

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

        MemUtils::acquireSemWait(_mem_config.mem_path_data_sem,
                             _data_sem,
                             _n_acq_trials,
                             _n_sem_acq_fail,
                             _journal,
                             _return_code,
                             1.0, // [s]
                             _force_reconnection,
                             _verbose,
                             _vlevel); // acquire shared data semaphore
        // Here to prevent access from any client (at this stage)

        _return_code = ReturnCode::RESET;

        MemUtils::checkMem(_mem_config.mem_path,
                            _data_shm_fd,
                            _journal,
                            _return_code,
                            _verbose,
                            _vlevel); // checks if memory was already allocated

        _return_code = ReturnCode::RESET;

        // data memory
        _initDataMem();

        // auxiliary data
        _initMetaMem();

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

            _acquireSemWait(_mem_config.mem_path_server_sem,
                        _srvr_sem); // blocking


            _releaseSem(_mem_config.mem_path_data_sem,
                        _data_sem);

            // set the running flag to true
            _running = true;
            _isrunning_view(0, 0) = 1; // for the clients

        }

    }

    template <typename Scalar>
    void Server<Scalar>::stop()
    {

        if (isRunning()) {

            _running = false;
            _isrunning_view(0, 0) = 0; // for the clients

            MemUtils::releaseSem(_mem_config.mem_path_server_sem,
                                _srvr_sem,
                                _journal,
                                _return_code,
                                _verbose,
                                _vlevel);

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

        _cleanMems(); // cleans up all memory,
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
    int Server<Scalar>::getNClients() {

        _acquireData();

        n_clients = _n_clients_view(0, 0);

        _releaseData();

        return n_clients;
    }

    template <typename Scalar>
    bool Server<Scalar>::writeTensor(const Tensor<Scalar>& data,
                                 int row,
                                 int col) {

        if (_running) {

            if (_acquireData()) { // non-blocking

                MemUtils::write(data,
                            _tensor_view,
                            row, col,
                            _journal,
                            _return_code,
                            false,
                            _vlevel);

                _releaseData();

                return true;
            }

        }

        if (!_running && _verbose) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP); // nonblocking

        }

        return false;

    }

    template <typename Scalar>
    bool Server<Scalar>::readTensor(Tensor<Scalar>& output,
                                    int row, int col) {

        if (_running) {

            if (_acquireData()) { // non-blocking

                MemUtils::read(row, col,
                               output,
                               _tensor_view,
                               _journal,
                               _return_code,
                               false,
                               _vlevel);

                _releaseData();

                return true;

            }

        }

        if (!_running && _verbose) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP); // nonblocking

        }

        return false;

    }

//    template <typename Scalar>
//    const MMap<Scalar>& Server<Scalar>::getTensorView() {

//        if (!_running && _verbose) {

//            std::string error = std::string("Server is not running. ") +
//                    std::string("Did you remember to call the run() method?");

//            _journal.log(__FUNCTION__,
//                 error,
//                 LogType::EXCEP);

//        }

//        return _tensor_view;

//    }

    template <typename Scalar>
    void Server<Scalar>::_acquireSemWait(const std::string& sem_path,
                                     sem_t*& sem)
    {
        _return_code = ReturnCode::RESET;

        MemUtils::acquireSemWait(sem_path,
                             sem,
                             _n_acq_trials,
                             _n_sem_acq_fail,
                             _journal,
                             _return_code,
                             1.0, // [s]
                             _force_reconnection,
                             false, // no verbosity (this is called very frequently)
                             _vlevel);

        _return_code = ReturnCode::RESET;

        if (isin(ReturnCode::SEMACQFAIL, _return_code)) {

            MemUtils::failWithCode(_return_code,
                                   _journal);

        }

    }

    template <typename Scalar>
    bool Server<Scalar>::_acquireSemRt(const std::string& sem_path,
                                     sem_t*& sem)
    {
        _return_code = ReturnCode::RESET;

        MemUtils::acquireSemTry(sem_path,
                             sem,
                             _journal,
                             _return_code,
                             _verbose,
                             VLevel::V0); // minimal verbosity (if enabled at all)

        if (isin(ReturnCode::SEMACQFAIL,
                 _return_code)) {

            return false;

        }

        _return_code = ReturnCode::RESET;

        return true;

    }

    template <typename Scalar>
    void Server<Scalar>::_releaseSem(const std::string& sem_path,
                                     sem_t*& sem)
    {
        _return_code = ReturnCode::RESET;

        MemUtils::releaseSem(sem_path,
                             sem,
                             _journal,
                             _return_code,
                             false, // no verbosity (this is called very frequently)
                             _vlevel);

        _return_code = ReturnCode::RESET;

        if (isin(ReturnCode::SEMRELFAIL, _return_code)) {

            MemUtils::failWithCode(_return_code,
                                   _journal);

        }

    }

    template <typename Scalar>
    bool Server<Scalar>::_acquireData(bool blocking)
    {

        if (blocking) {

            _acquireSemWait(_mem_config.mem_path_data_sem,
                            _data_sem); // this is blocking

            return true;

        }
        else {

            return _acquireSemRt(_mem_config.mem_path_data_sem,
                        _data_sem);

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_releaseData()
    {

        _releaseSem(_mem_config.mem_path_data_sem,
                    _data_sem);

    }

    template <typename Scalar>
    void Server<Scalar>::_cleanMetaMem()
    {
        // closing file descriptors and also unlinking
        // memory

        _return_code = ReturnCode::RESET;

        MemUtils::cleanUpMem(_mem_config.mem_path_nrows,
                             _nrows_shm_fd,
                             _journal,
                             _return_code,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem(_mem_config.mem_path_ncols,
                             _ncols_shm_fd,
                             _journal,
                             _return_code,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem(_mem_config.mem_path_clients_counter,
                             _n_clients_shm_fd,
                             _journal,
                             _return_code,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem(_mem_config.mem_path_dtype,
                             _dtype_shm_fd,
                             _journal,
                             _return_code,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        MemUtils::cleanUpMem(_mem_config.mem_path_isrunning,
                             _isrunning_shm_fd,
                             _journal,
                             _return_code,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        _return_code = ReturnCode::RESET;

    }

    template <typename Scalar>
    void Server<Scalar>::_cleanMems()
    {

        if (!_terminated) {

            _return_code = ReturnCode::RESET;

            MemUtils::cleanUpMem(_mem_config.mem_path,
                                 _data_shm_fd,
                                 _journal,
                                 _return_code,
                                 _verbose,
                                 _vlevel);

            _return_code = ReturnCode::RESET;

            _cleanMetaMem();

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
    void Server<Scalar>::_initMetaMem()
    {
        _return_code = ReturnCode::RESET; // resets return code

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_nrows,
                        _nrows_shm_fd,
                        _n_rows_view,
                        _journal,
                        _return_code,
                        _verbose,
                        _vlevel);

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_ncols,
                        _ncols_shm_fd,
                        _n_cols_view,
                        _journal,
                        _return_code,
                        _verbose,
                        _vlevel);

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_clients_counter,
                        _n_clients_shm_fd,
                        _n_clients_view,
                        _journal,
                        _return_code,
                        _verbose,
                        _vlevel);

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_dtype,
                        _dtype_shm_fd,
                        _dtype_view,
                        _journal,
                        _return_code,
                        _verbose,
                        _vlevel);

        MemUtils::initMem<bool>(1,
                        1,
                        _mem_config.mem_path_isrunning,
                        _isrunning_shm_fd,
                        _isrunning_view,
                        _journal,
                        _return_code,
                        _verbose,
                        _vlevel);

        if (!isin(ReturnCode::MEMCREATFAIL,
                _return_code) &&
            !isin(ReturnCode::MEMSETFAIL,
                _return_code) &&
            !isin(ReturnCode::MEMMAPFAIL,
                 _return_code)) {

            // all memory creations where successful

            _n_rows_view(0, 0) = n_rows;
            _n_cols_view(0, 0) = n_cols;
            _n_clients_view(0, 0) = 0; // to be improved
            // (what happens when server crashes and clients remain appended?)
            _isrunning_view(0, 0) = 0;

            _dtype_view(0, 0) = sizeof(Scalar);

            _return_code = ReturnCode::RESET;
        }
        else {

            MemUtils::failWithCode(_return_code,
                                   _journal);

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_initDataMem()
    {

        _return_code = _return_code + ReturnCode::RESET;

        if (!isin(ReturnCode::MEMCREATFAIL,
                _return_code) &&
            !isin(ReturnCode::MEMSETFAIL,
                _return_code) &&
            !isin(ReturnCode::MEMMAPFAIL,
                 _return_code)) {

            MemUtils::initMem<Scalar>(n_rows,
                            n_cols,
                            _mem_config.mem_path,
                            _data_shm_fd,
                            _tensor_view,
                            _journal,
                            _return_code,
                            _verbose,
                            _vlevel);

            _return_code = _return_code + ReturnCode::RESET;

        }
        else {

            MemUtils::failWithCode(_return_code,
                                   _journal);

        }

    }

    template <typename Scalar>
    void Server<Scalar>::_initSems()
    {

        MemUtils::initSem(_mem_config.mem_path_server_sem,
                          _srvr_sem,
                          _journal,
                          _return_code,
                          _verbose,
                          _vlevel);

        MemUtils::initSem(_mem_config.mem_path_data_sem,
                          _data_sem,
                          _journal,
                          _return_code,
                          _verbose,
                          _vlevel);

    }

    template <typename Scalar>
    void Server<Scalar>::_closeSems()
    {
        // closes semaphores and also unlinks it
        // Other processes who had it open can still use it, but no new
        // process can access it
        MemUtils::closeSem(_mem_config.mem_path_server_sem,
                           _srvr_sem,
                           _journal,
                           _return_code,
                           _verbose,
                           _vlevel,
                           true);

        MemUtils::closeSem(_mem_config.mem_path_data_sem,
                           _data_sem,
                           _journal,
                           _return_code,
                           _verbose,
                           _vlevel,
                           true);

    }

    template <typename Scalar>
    std::string Server<Scalar>::_getThisName()
    {

        return _this_name;
    }

    // explicit instantiations for specific supported types
    template class Server<double>;
    template class Server<float>;
    template class Server<int>;
    template class Server<bool>;
}
