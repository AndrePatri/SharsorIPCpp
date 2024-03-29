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

#ifndef CONDVAR_HPP
#define CONDVAR_HPP

#include <chrono>
#include <thread>
#include <memory>

#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

// public headers
#include <SharsorIPCpp/SharedMemConfig.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>
#include <SharsorIPCpp/ReturnCodes.hpp>

namespace SharsorIPCpp{

    class ConditionVariable{

        using VLevel = Journal::VLevel;
        using LogType = Journal::LogType;
        
        using NamedCondition = boost::interprocess::named_condition;
        using NamedMutex = boost::interprocess::named_mutex;
        using ScopedLock = boost::interprocess::scoped_lock<NamedMutex>;

        public:

            typedef std::weak_ptr<ConditionVariable> WeakPtr;
            typedef std::shared_ptr<ConditionVariable> Ptr;
            typedef std::unique_ptr<ConditionVariable> UniquePtr;

            ConditionVariable(bool is_server,
                    std::string basename,
                    std::string name_space = "",
                    bool verbose = false,
                    VLevel vlevel = VLevel::V0);

            ~ConditionVariable();

            void wait();

            void notify_one();

            void notify_all();

            void close();
            
        private:

            bool _verbose = false;

            bool _is_server = false;

            std::string _basename, _namespace;

            std::string _this_name = "SharsorIPCpp::ConditionVariable";

            VLevel _vlevel = VLevel::V0; // minimal debug info

            Journal _journal; // for rt-friendly logging

            std::string _getThisName(); // used to get this class
            // name

            SharedMemConfig _mem_config;

            NamedCondition _named_cond;
            NamedMutex _named_mutex;

    };

}

#endif // CLIENT_HPP
