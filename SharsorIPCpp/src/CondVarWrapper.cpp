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

#include <SharsorIPCpp/CondVarWrapper.hpp>

namespace SharsorIPCpp {

    ConditionWrapper::ConditionWrapper(
                   bool is_server,
                   std::string basename,
                   std::string name_space,
                   bool verbose,
                   VLevel vlevel)
        : _is_server(is_server),
        _verbose(verbose),
        _vlevel(vlevel),
        _journal(Journal(_getThisName())),
        _cond_variable(is_server,
                basename,
                name_space,
                verbose,
                vlevel)
        
    {

    }

    ConditionWrapper::~ConditionWrapper(){
        
        close();
    }

    void ConditionWrapper::close() {

        

        _closed = true;
    
    }

    std::string ConditionWrapper::_getThisName(){

        return _this_name;
    }

    bool ConditionWrapper::notify(std::function<bool()> pred,
                        bool notify_all) {

        ScopedLock data_lock = _cond_variable.lock(); // locks condition mutex

        bool success = pred(); // call custom function

        // ConditionVariable::unlock(data_lock); // release mutex

        if (success && notify_all) {
            
            _cond_variable.notify_all(); // notify all listeners on condition variable

        }
        else if (success && !notify_all) {

            _cond_variable.notify_one(); // notify only one

        }
        else {
            
            if (_verbose &&
                _vlevel > VLevel::V0)
            {
                std::string warn = std::string("[") + _cond_variable.cond_var_path() + std::string("] ") +
                    std::string("Provided predicate failed. No notification will be sent");

                _journal.log(__FUNCTION__,
                    warn,
                    LogType::WARN);
            }

        }
        
        return success;
    }

    bool ConditionWrapper::wait(std::function<bool()> pred,
                            unsigned int ms){

        ScopedLock data_lock = _cond_variable.lock(); // locks condition mutex

        bool verified = pred(); // check condition in case it was already verified

        if (!verified) {

            // wait until the condition is verified
            verified = _cond_variable.timedwait_for(data_lock, 
                                ms, 
                                pred); // locks back the mutex atomically when returning
            // verified will be false only if the timeout was reached
        }

        // ConditionVariable::unlock(data_lock); // release mutex

        if (!verified) {

            if (_verbose &&
                _vlevel > VLevel::V0)
            {
                std::string warn = std::string("Condition at ") + _cond_variable.cond_var_path() +
                    std::string(" not verified within the timeout of ") + 
                    std::to_string(ms) + 
                    std::string(" ms");

                _journal.log(__FUNCTION__,
                    warn,
                    LogType::WARN);
            }
        }
        
        return verified;

    }

    void ConditionWrapper::wait(std::function<bool()> pred){

        ScopedLock data_lock = _cond_variable.lock(); // locks condition mutex

        bool verified = pred(); // check condition in case it was already verified

        if (!verified) {

            // wait until the condition is verified
            _cond_variable.wait_for(data_lock, 
                                pred); // locks back the mutex atomically when returning
        }

        ConditionVariable::unlock(data_lock); // release mutex
        
    }

}