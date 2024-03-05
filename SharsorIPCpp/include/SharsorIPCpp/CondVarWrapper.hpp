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

#ifndef CONDVARWRAPPER_HPP
#define CONDVARWRAPPER_HPP

#include <chrono>
#include <thread>
#include <memory>

#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// public headers
#include <SharsorIPCpp/SharedMemConfig.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>
#include <SharsorIPCpp/ReturnCodes.hpp>

#include <SharsorIPCpp/CondVar.hpp>

namespace SharsorIPCpp{

    class ConditionWrapper{

        using VLevel = Journal::VLevel;
        using LogType = Journal::LogType;
        using ConditionVariable = SharsorIPCpp::ConditionVariable;
        using ScopedLock = ConditionVariable::ScopedLock;

        public:

            typedef std::weak_ptr<ConditionWrapper> WeakPtr;
            typedef std::shared_ptr<ConditionWrapper> Ptr;
            typedef std::unique_ptr<ConditionWrapper> UniquePtr;

            ConditionWrapper(bool is_server,
                    std::string basename,
                    std::string name_space = "",
                    bool verbose = false,
                    VLevel vlevel = VLevel::V0);

            ~ConditionWrapper();
            
            bool notify(std::function<bool()> pred,
                bool notify_all = true);
            
            bool wait(std::function<bool()> pred, 
                unsigned int ms = 1000);

            void close();

        private:

            bool _verbose = false;

            bool _is_server = false;
            
            bool _closed = false;
            
            std::string _this_name = "SharsorIPCpp::ConditionWrapper";

            VLevel _vlevel = VLevel::V0; // minimal debug info

            Journal _journal; // for rt-friendly logging

            std::string _getThisName(); // used to get this class
            // name

            ConditionVariable _cond_variable;

    };

}

#endif // CONDVARWRAPPER_HPP
