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

#ifndef CONSUMER_HPP
#define CONSUMER_HPP

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
#include <SharsorIPCpp/Client.hpp>

namespace SharsorIPCpp{

    class Consumer{

        using VLevel = Journal::VLevel;
        using LogType = Journal::LogType;
        using ConditionVariable = SharsorIPCpp::ConditionVariable;
        using ScopedLock = ConditionVariable::ScopedLock;
        using SharedCounter = SharsorIPCpp::Client<int>;
        using CounterView = SharsorIPCpp::Tensor<int>;

        public:

            typedef std::weak_ptr<Consumer> WeakPtr;
            typedef std::shared_ptr<Consumer> Ptr;
            typedef std::unique_ptr<Consumer> UniquePtr;

            Consumer(std::string basename,
                    std::string name_space = "",
                    bool verbose = false,
                    VLevel vlevel = VLevel::V0);

            ~Consumer();

            void run();

            void close();

            bool wait(int ms_timeout = -1);
            bool wait_and_ack(std::function<bool()> pre_ack,
                    int ms_timeout = -1);

            bool ack();

        private:

            bool _verbose = false;
            
            bool _closed = false;
            bool _is_running = false;

            bool _trigger_received = false;

            bool _timeout = false;

            int _internal_trigger_counter = 0;
            int _trigger_counter_increment = 0;

            int _fail_count = 0;

            std::string _basename, _namespace;
            
            std::string THISNAME = "SharsorIPCpp::Consumer";
            
            std::string TRIGGER_COND_NAME = "TriggerCond";
            std::string ACK_COND_NAME = "AckCond";

            std::string TRIGGER_BASENAME = "Trigger";
            std::string ACK_BASENAME = "Ack";

            VLevel _vlevel = VLevel::V0; // minimal debug info

            Journal _journal; // for rt-friendly logging

            ConditionVariable _trigger_cond;
            ConditionVariable _ack_cond;

            SharedCounter _trigger_counter_clnt;
            CounterView _trigger_counter;

            SharedCounter _ack_counter_clnt;
            CounterView _ack_counter; 

            std::string _getThisName(); // used to get this class
            // name

            void _check_running(std::string calling_method);

            bool _check_trigger_received();

            bool _wait(ScopedLock& ack_lock, int ms_timeout = -1);

            bool _acknowledge();

    };

}

#endif // CONSUMER_HPP
