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
#ifndef PYSERVER_HPP
#define PYSERVER_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <PySharsorIPC/PyDTypes.hpp>

#include <PySharsorIPCUtils.hpp>
#include <WrapUtils.hpp>

namespace PySharsorIPC{

    namespace PyServer {

        using VLevel = SharsorIPCpp::Journal::VLevel;
        using LogType = SharsorIPCpp::Journal::LogType;
        using DType = SharsorIPCpp::DType;

        pybind11::object ServerFactory(int n_rows,
                                int n_cols,
                                std::string basename = "MySharedMemory",
                                std::string name_space = "",
                                bool verbose = false,
                                VLevel vlevel = VLevel::V0,
                                bool safe = true,
                                bool force_reconnection = false,
                                SharsorIPCpp::DType dtype = SharsorIPCpp::DType::Float,
                                int layout = SharsorIPCpp::ColMajor);

        void bind_ServerWrapper(pybind11::module& m);

        template <typename Scalar,
                int Layout = SharsorIPCpp::MemLayoutDefault>
        void bindServerT(pybind11::module &m, const char* name);

        void bindServers(pybind11::module &m);

        void bindServerFactory(pybind11::module &m,
                            const char* name = "ServerFactory");

    }

}

#endif // PYSERVER_HPP
