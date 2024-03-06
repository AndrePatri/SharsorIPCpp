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

#include <SharsorIPCpp/Consumer.hpp>

namespace SharsorIPCpp {

    Consumer::Consumer(
                std::string basename,
                std::string name_space,
                bool verbose,
                VLevel vlevel)
        : _verbose(verbose),
        _vlevel(vlevel),
        _journal(Journal(_getThisName())),
        _trigger_cond(true,
                basename + _trigger_cond_name,
                name_space,
                verbose,
                vlevel),
        _ack_cond(true,
                basename + _ack_cond_name,
                name_space,
                verbose,
                vlevel),
        _trigger_counter_clnt(basename + _trigger_basename, 
            name_space,
            verbose,
            vlevel,
            false),
        _trigger_counter(1, 1),
        _ack_counter_clnt(basename + _ack_basename, 
            name_space,
            verbose,
            vlevel,
            false),
        _ack_counter(1, 1),
        _closed(true)
    {

    }

    Consumer::~Consumer(){
        
        close();
    }

    void Consumer::run() {

        if (!_is_running) {
            
            _trigger_counter_clnt.attach();
            _ack_counter_clnt.attach();
            
            _is_running = true;
            _closed = false;

            _internal_trigger_counter = 0;
        }
    }

    void Consumer::close() {

        if (!_closed) {
            
            _trigger_counter_clnt.close();
            _ack_counter_clnt.close();

            _closed = true;
        }
    }

    bool Consumer::wait(unsigned int ms_timeout) {

        _check_running(std::string(__FUNCTION__));

        ScopedLock trigger_lock = _trigger_cond.lock();

        _trigger_received = false;

        while (!_trigger_received) {

            _trigger_received = _check_trigger_received();

            if (!_trigger_received) {
                
                // wait (unlock and lock mutex atomically when
                // returning)
                if(!_wait(trigger_lock, ms_timeout)) {

                    return false;
                }

            }
        }

        return true;
    }

    bool Consumer::wait_and_ack(std::function<bool()> pre_ack,
                    unsigned int ms_timeout) {

        _fail_count = 0;
        
        if (!wait(ms_timeout)) {

            _fail_count += 1;

        } else {

            if (!pre_ack()) {

                _fail_count += 1;
            }

            if (!ack()) { 

                _fail_count += 1;

            }

        } 

        return _fail_count == 0;
    }

    bool Consumer::ack() {

        _check_running(std::string(__FUNCTION__));

        ScopedLock ack_lock = _ack_cond.lock();

        return _acknowledge();

    }

    bool Consumer::_acknowledge() {
        
        _fail_count = 0;

        if (!_ack_counter_clnt.read(_ack_counter, 0, 0)) {

            _journal.log(__FUNCTION__,
                "Could not read acknowledge counter!",
                LogType::EXCEP, 
                false); // throw exception

            _fail_count += 1;

        }

        _ack_counter.array() += 1; // increment shared ack counter and write to memory
        
        if (!_ack_counter_clnt.write(_ack_counter, 0, 0)) {

            _journal.log(__FUNCTION__,
                "Could not write acknowledge counter!",
                LogType::EXCEP, 
                false); // throw exception

            _fail_count += 1;
            
        }

        _ack_cond.notify_one();

        return _fail_count == 0;
        
    }

    bool Consumer::_check_trigger_received() {

        _trigger_counter_clnt.read(_trigger_counter, 0, 0); // reads current value
        // of trigger counter (only written by Producer)

        _trigger_counter_increment = _trigger_counter(0, 0) - _internal_trigger_counter;

        if (_trigger_counter_increment > 1 || 
            _trigger_counter_increment < 0) {

            _journal.log(__FUNCTION__,
                "Found trigger increment < 0 or > 1.",
                LogType::EXCEP, 
                true); // throw exception

        }

        if (_trigger_counter_increment == 1) {
            
            _internal_trigger_counter = _trigger_counter(0, 0);

            return true;

        } else {
            
            return false;
        }

    }
    
    bool Consumer::_wait(ScopedLock& lock, 
                unsigned int ms_timeout) {

        if (ms_timeout > 0) {

            _timeout = !(_trigger_cond.timedwait(lock, ms_timeout)); // wait with timeout
            
            return !_timeout;

        } else {

            _trigger_cond.wait(lock); // blocking

            return true;
        }

    }

    std::string Consumer::_getThisName(){

        return _this_name;
    }

    void Consumer::_check_running(std::string calling_method) {

        if (!_is_running) {

            _journal.log(calling_method,
                "Not running. Did you call the run() method?",
                LogType::EXCEP, 
                true); // throw exception

        }
    }

}