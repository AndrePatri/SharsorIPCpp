// Copyright (C) 2023  Andrea Patrizi (AndrePatri)
// 
// This file is part of SharsorIPCpp and distributed under the General Public License version 2 license.
// 
// SharsorIPCpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// SharsorIPCpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with SharsorIPCpp.  If not, see <http://www.gnu.org/licenses/>.
// 
#include <iostream>
#include <Eigen/Dense>
#include <cstring>
#include <typeinfo>
#include <ctime>

#include <SharsorIPCpp/Client.hpp>

// private headers
#include <MemUtils.hpp>

namespace SharsorIPCpp {

    template <typename Scalar, int Layout>
    Client<Scalar, Layout>::Client(
                   std::string basename,
                   std::string name_space,
                   bool verbose,
                   VLevel vlevel)
        : _mem_config(basename, name_space),
          _verbose(verbose),
          _vlevel(vlevel),
          _tensor_view(nullptr,
                       -1,
                       -1),
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
          _mem_layout_view(nullptr,
                      1,
                      1),
          _journal(Journal(_getThisName()))
    {

        static_assert(MemUtils::IsValidDType<Scalar>::value,
                "Invalid data type provided.");


        _terminated = false; // just in case

    }

    template <typename Scalar, int Layout>
    Client<Scalar, Layout>::~Client() {

        if (!_terminated) {

            close();
        }

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::attach()
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
        _acquireData(true); // blocking

        _checkDType(); // checks data type consistency

        _checkMemLayout(); // checks memory layout consistency

        _n_rows = _n_rows_view(0, 0);
        _n_cols = _n_cols_view(0, 0);
        _n_clients_view(0, 0) = _n_clients_view(0, 0) + 1; // increase clients counter

        // we have now all the info to create the shared tensor
        _initDataMem();

        _tensor_copy = Tensor<Scalar, Layout>::Zero(_n_rows,
                                            _n_cols); // used to hold
        // a copy of the shared tensor data

        // releasing data semaphore so that other clients/the server can access the tensor
        _releaseData();

        _attached = true;
    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::detach()
    {
        if (_attached) {

            _acquireData(); // blocking

            _n_clients_view(0, 0) = _n_clients_view(0, 0) - 1; // increase clients counter

            // releasing data semaphore so that other clients/the server can access the tensor
            _releaseData();

            _attached = false;

        }
    }

    template <typename Scalar, int Layout>
    bool Client<Scalar, Layout>::isAttached()
    {

        return _attached;

    }

    template <typename Scalar, int Layout>
    int Client<Scalar, Layout>::getNRows()
    {

        return _n_rows;

    }

    template <typename Scalar, int Layout>
    int Client<Scalar, Layout>::getNCols()
    {

        return _n_cols;

    }

    template <typename Scalar, int Layout>
    DType Client<Scalar, Layout>::getScalarType() const {

        return CppTypeToDType<Scalar>::value;

    }

    template <typename Scalar, int Layout>
    int Client<Scalar, Layout>::getMemLayout() const {

        return _mem_layout;

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::close()
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

    template <typename Scalar, int Layout>
    bool Client<Scalar, Layout>::write(const TRef<Scalar, Layout> data,
                                 int row,
                                 int col) {

        if (_attached) {

            if(_acquireData()) {

                bool success_write = MemUtils::write<Scalar, Layout>(
                                        data,
                                        _tensor_view,
                                        row, col,
                                        _journal,
                                        _return_code,
                                        false,
                                        _vlevel);

                _releaseData();

                return success_write;

            } else {

                return false; // failed to acquire sem
            }

        }

        if (!_attached && _verbose) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP); // nonblocking

        }

        return false;

    }

    template <typename Scalar, int Layout>
    bool Client<Scalar, Layout>::write(const TensorView<Scalar, Layout>& data,
                                     int row,
                                     int col) {

        if (_attached) {

            if(_acquireData()) {

                bool success_write = MemUtils::write<Scalar, Layout>(
                                        data,
                                        _tensor_view,
                                        row, col,
                                        _journal,
                                        _return_code,
                                        false,
                                        _vlevel);

                _releaseData();

                return success_write;

            } else {

                return false; // failed to acquire sem
            }

        }

        if (!_attached && _verbose) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP); // nonblocking

        }

         return false;

    }

    template <typename Scalar, int Layout>
    bool Client<Scalar, Layout>::read(TRef<Scalar, Layout> output,
                                    int row, int col) {

        if (_attached) {

            if(_acquireData()) {

                bool success_read = MemUtils::read<Scalar, Layout>(
                            row, col,
                            output,
                            _tensor_view,
                            _journal,
                            _return_code,
                            false,
                            _vlevel);

                _releaseData();

                return success_read;

            } else {

                return false; // failed to acquire sem
            }

        }

        if (!_attached && _verbose) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP);

        }

        return false;

    }

    template <typename Scalar, int Layout>
    bool Client<Scalar, Layout>::read(TensorView<Scalar, Layout>& output,
                                    int row, int col) {

        if (_attached) {

            if(_acquireData()) {

                bool success_read = MemUtils::read<Scalar, Layout>(
                                       row, col,
                                       output,
                                       _tensor_view,
                                       _journal,
                                       _return_code,
                                       false,
                                       _vlevel);

                _releaseData();

                return success_read;

            } else {

                return false; // failed to acquire sem

            }

        }

        if (!_attached && _verbose) {

            std::string error = std::string("Server is not running. ") +
                    std::string("Did you remember to call the run() method?");

            _journal.log(__FUNCTION__,
                 error,
                 LogType::EXCEP); // nonblocking

        }

        return false;

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_waitForServer()
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

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_checkDType()
    {
        if (_dtype_view(0, 0) != sizeof(Scalar)) {

            // not impeccable: different types may have in general different sizes

            std::string error = std::string("Client initialized with element size of ") +
                    std::to_string(sizeof(Scalar)) +
                    std::string(", while the Server was initialized with size ") +
                    std::to_string(_dtype_view(0, 0));

            _journal.log(__FUNCTION__,
                         error,
                         LogType::EXCEP,
                         true); // actually raise exception

        }
    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_checkMemLayout()
    {
        if (_mem_layout_view(0, 0) != _mem_layout) {

            std::string error = std::string("Client initialized with memory layout ") +
                    MemUtils::getLayoutName(_mem_layout) +
                    std::string(", while the Server was initialized with layout ") +
                    MemUtils::getLayoutName(_mem_layout_view(0, 0));

            _journal.log(__FUNCTION__,
                         error,
                         LogType::EXCEP,
                         true); // actually raise exception

        }

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_acquireSemWait(const std::string& sem_path,
                                     sem_t*& sem)
    {
        _return_code = _return_code + ReturnCode::RESET;


        MemUtils::acquireSemWait(sem_path,
                             sem,
                             _n_acq_trials,
                             _n_sem_acq_fail,
                             _journal,
                             _return_code,
                             1.0, // [s]
                             false,
                             false, // no verbosity (this is called very frequently)
                             _vlevel);

        _return_code = _return_code + ReturnCode::RESET;


        if (isin(ReturnCode::SEMACQFAIL, _return_code)) {

            MemUtils::failWithCode(_return_code,
                                   _journal);

        }

    }

    template <typename Scalar, int Layout>
    bool Client<Scalar, Layout>::_acquireSemRt(const std::string& sem_path,
                                     sem_t*& sem)
    {
        _return_code = _return_code + ReturnCode::RESET;


        MemUtils::acquireSemTry(sem_path,
                             sem,
                             _journal,
                             _return_code,
                             _verbose,
                             VLevel::V0);

        if (isin(ReturnCode::SEMACQFAIL,
                 _return_code)) {

            return false;

        }

        _return_code = _return_code + ReturnCode::RESET;


        return true;

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_releaseSem(const std::string& sem_path,
                                     sem_t*& sem)
    {
        _return_code = _return_code + ReturnCode::RESET;


        MemUtils::releaseSem(sem_path,
                             sem,
                             _journal,
                             _return_code,
                             false, // no verbosity (this is called very frequently)
                             _vlevel);

        _return_code = _return_code + ReturnCode::RESET;


        if (isin(ReturnCode::SEMRELFAIL, _return_code)) {

            MemUtils::failWithCode(_return_code,
                                   _journal);

        }

    }

    template <typename Scalar, int Layout>
    bool Client<Scalar, Layout>::_acquireData(bool blocking)
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

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_releaseData()
    {

        _releaseSem(_mem_config.mem_path_data_sem,
                    _data_sem);

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_cleanMetaMem()
    {
        // closing file descriptors and but not unlinking
        // memory

        _return_code = _return_code + ReturnCode::RESET;


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

        MemUtils::cleanUpMem(_mem_config.mem_path_mem_layout,
                             _mem_layout_shm_fd,
                             _journal,
                             _return_code,
                             _verbose,
                             _vlevel,
                             _unlink_data);

        _return_code = _return_code + ReturnCode::RESET;


    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_cleanMems()
    {

        if (!_terminated) {

            _return_code = _return_code + ReturnCode::RESET;


            MemUtils::cleanUpMem(_mem_config.mem_path,
                                 _data_shm_fd,
                                 _journal,
                                 _return_code,
                                 _verbose,
                                 _vlevel,
                                 false); // closing but no unlinking

            _return_code = _return_code + ReturnCode::RESET;


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

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_initDataMem()
    {

        _return_code = _return_code + ReturnCode::RESET;

        if (!isin(ReturnCode::MEMCREATFAIL,
                _return_code) &&
            !isin(ReturnCode::MEMSETFAIL,
                _return_code) &&
            !isin(ReturnCode::MEMMAPFAIL,
                 _return_code)) {

            MemUtils::initMem<Scalar, Layout>(_n_rows,
                            _n_cols,
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

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_initMetaMem()
    {
        _return_code = _return_code + ReturnCode::RESET;
 // resets return code

        // auxiliary data
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

        MemUtils::initMem<int>(1,
                        1,
                        _mem_config.mem_path_mem_layout,
                        _mem_layout_shm_fd,
                        _mem_layout_view,
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

            _return_code = _return_code + ReturnCode::RESET;


        }
        else {

            MemUtils::failWithCode(_return_code,
                                   _journal);
        }

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_initSems()
    {

        MemUtils::initSem(_mem_config.mem_path_data_sem,
                          _data_sem,
                          _journal,
                          _return_code,
                          _verbose,
                          _vlevel);

    }

    template <typename Scalar, int Layout>
    void Client<Scalar, Layout>::_closeSems()
    {
        // closes semaphores but doesn't unlink them
        MemUtils::closeSem(_mem_config.mem_path_data_sem,
                           _data_sem,
                           _journal,
                           _return_code,
                           _verbose,
                           _vlevel,
                           false);

    }

    template <typename Scalar, int Layout>
    std::string Client<Scalar, Layout>::_getThisName()
    {

        return _this_name;
    }

    // explicit instantiations for specific supported types
    // explicit instantiations for specific supported types
    // and layouts
    template class Client<double, ColMajor>;
    template class Client<float, ColMajor>;
    template class Client<int, ColMajor>;
    template class Client<bool, ColMajor>;

    template class Client<double, RowMajor>;
    template class Client<float, RowMajor>;
    template class Client<int, RowMajor>;
    template class Client<bool, RowMajor>;
}

