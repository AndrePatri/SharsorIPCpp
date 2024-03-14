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

#include <SharsorIPCpp/Producer.hpp>

namespace SharsorIPCpp {

    Producer::Producer(
                std::string basename,
                std::string name_space,
                bool verbose,
                VLevel vlevel,
                bool force_reconnection)
        : _verbose(verbose),
        _vlevel(vlevel),
        _journal(Journal(_getThisName())),
        _trigger_cond(true,
                basename + TRIGGER_COND_NAME,
                name_space,
                verbose,
                vlevel),
        _ack_cond(true,
                basename + ACK_COND_NAME,
                name_space,
                verbose,
                vlevel),
        _trigger_counter_srvr(1, 1,
            basename + TRIGGER_BASENAME, 
            name_space,
            verbose,
            vlevel,
            force_reconnection, 
            false),
        _trigger_counter(1, 1),
        _ack_counter_srvr(1, 1,
            basename + ACK_BASENAME, 
            name_space,
            verbose,
            vlevel,
            force_reconnection, 
            false),
        _ack_counter(1, 1),
        _closed(true),
        _basename(basename),
        _namespace(name_space)
    {

    }

    Producer::~Producer(){
        
        close();
    }

    void Producer::run() {

        if (!_is_running) {

            _trigger_counter_srvr.run();
            _ack_counter_srvr.run();

            _init_counters();

            _is_running = true;
            _closed = false;

            _acks_before = 0;

            if (_verbose &&
                _vlevel > VLevel::V1) {

                std::string info = std::string("Producer ")+_basename+std::string("-")+_namespace  
                    + std::string(" transitioned to running state.");

                _journal.log(__FUNCTION__,
                    info,
                    LogType::STAT);

            }
        }

    }

    void Producer::close() {

        if (!_closed) {
            
            _trigger_counter_srvr.close();
            _ack_counter_srvr.close();

            _closed = true;
        }
        
    
    }

    void Producer::trigger() {

        _check_running(std::string(__FUNCTION__));

        ScopedLock trigger_lock = _trigger_cond.lock();
        
        _increment_trigger();

        _trigger_cond.notify_all();
    }

    bool Producer::wait_ack_from(int n_consumers,
                            int ms_timeout) {
        
        _check_running(std::string(__FUNCTION__));

        ScopedLock ack_lock = _ack_cond.lock();
        
        _ack_completed = false;

        while (!_ack_completed) {
            
            _ack_completed = _check_ack_counter(n_consumers);
            
            if (!_ack_completed) {
                
                // wait (unlock and lock mutex atomically when
                // returning)
                if(!_wait(ack_lock, ms_timeout)) {

                    return false;
                }
            }
            
        }

        return true;
        
    }
    
    void Producer::_init_counters() {

        ScopedLock trigger_lock = _trigger_cond.lock();
        _trigger_counter(0, 0) = 0; // initialize shared counter to 0
        if (!_trigger_counter_srvr.write(_trigger_counter, 0, 0)) {
            _journal.log(__FUNCTION__,
                "Could not initialize trigger counter!",
                LogType::EXCEP, 
                true); // throw exception
        }

        ScopedLock ack_lock = _ack_cond.lock();
        _ack_counter(0, 0) = 0; // initialize shared counter to 0
        if (!_ack_counter_srvr.write(_ack_counter, 0, 0)) {
            _journal.log(__FUNCTION__,
                "Could not initialize acknowledge counter!",
                LogType::EXCEP, 
                true); // throw exception
        }

    }

    void Producer::_increment_trigger() {

        _trigger_counter.array() += 1; // increment counter 
        if (!_trigger_counter_srvr.write(_trigger_counter, 0, 0)) {
            _journal.log(__FUNCTION__,
                "Could not increment trigger counter!",
                LogType::EXCEP, 
                true); // throw exception
        }

    }

    bool Producer::_check_ack_counter(int n_consumers) {

        if (!_ack_counter_srvr.read(_ack_counter, 0, 0)) {

            _journal.log(__FUNCTION__,
                "Could not read acknowledge counter!",
                LogType::EXCEP, 
                true); // throw exception
        }

        if ((_ack_counter(0, 0) - _acks_before) == n_consumers) {
            
            _acks_before = _ack_counter(0, 0);
            
            return true;

        } else {

            return false;
        }

    }

    bool Producer::_wait(ScopedLock& ack_lock, 
                int ms_timeout) {

        if (ms_timeout > 0) {

            _timeout = !(_ack_cond.timedwait(ack_lock, ms_timeout)); // wait with timeout
            
            return !_timeout;

        } else {

            _ack_cond.wait(ack_lock); // blocking

            return true;
        }

    }
    
    std::string Producer::_getThisName(){

        return THISNAME;
    }

    void Producer::_check_running(std::string calling_method) {

        if (!_is_running) {

            _journal.log(calling_method,
                "Not running. Did you call the run() method?",
                LogType::EXCEP, 
                true); // throw exception

        }
    }

}