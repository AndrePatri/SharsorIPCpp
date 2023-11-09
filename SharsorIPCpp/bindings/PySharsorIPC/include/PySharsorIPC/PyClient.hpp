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
#ifndef PYCLIENT_HPP
#define PYCLIENT_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>

#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <PySharsorIPC/PyDTypes.hpp>
#include <WrapUtils.hpp>

namespace PySharsorIPC{

    namespace PyClient {

        using VLevel = SharsorIPCpp::Journal::VLevel;
        using LogType = SharsorIPCpp::Journal::LogType;
        using DType = SharsorIPCpp::DType;

        pybind11::object ClientFactory(std::string basename = "MySharedMemory",
                                std::string name_space = "",
                                bool verbose = false,
                                VLevel vlevel = VLevel::V0,
                                SharsorIPCpp::DType dtype = SharsorIPCpp::DType::Float,
                                int layout = SharsorIPCpp::ColMajor);

        void bind_ClientWrapper(pybind11::module& m);

        template <typename Scalar,
                int Layout = SharsorIPCpp::MemLayoutDefault>
        void bindClientT(pybind11::module &m, const char* name);

        void bindClients(pybind11::module &m);

        void bindClientFactory(pybind11::module &m,
                            const char* name = "ClientFactory");

    }

}


#endif // PYCLIENT_HPP
