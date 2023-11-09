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
#ifndef PYSTRINGTENSOR_HPP
#define PYSTRINGTENSOR_HPP

#include <pybind11/pybind11.h>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/StringTensor.hpp>

namespace py = pybind11;

namespace PySharsorIPC {

    namespace PyStringTensor{
        
        using namespace SharsorIPCpp;
 
        using VLevel = Journal::VLevel;
        using LogType = Journal::LogType;
        
        void declare_StringTensorServer(py::module &m);
        void declare_StringTensorClient(py::module &m);

    }

}

#endif // PYSTRINGTENSOR_HPP
