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
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <pybind11/numpy.h>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Server.hpp>

#include <PySharsorIPC/PyStringTensor.hpp>

namespace py = pybind11;
using namespace SharsorIPCpp;

void PySharsorIPC::PyStringTensor::declare_StringTensorServer(py::module &m) {

    std::string pyclass_name = std::string("StringTensorServer");

    py::class_<StringTensor<StrServer>>(m, pyclass_name.c_str())

        .def(py::init<int, std::string, std::string, bool, VLevel, bool>(),
             py::arg("length"),
             py::arg("basename") = "MySharedMemory",
             py::arg("name_space") = "",
             py::arg("verbose") = false,
             py::arg("vlevel") = VLevel::V0,
             py::arg("force_reconnection") = false)

        .def("run", &StringTensor<StrServer>::run)

        // Add other method bindings here ...
        .def("isRunning", &StringTensor<StrServer>::isRunning)

        .def("length", &StringTensor<StrServer>::getLength)

        .def("close", &StringTensor<StrServer>::close)

        .def("write_vec",
            (bool (StringTensor<StrServer>::*)(const std::vector<std::string>&, int))
            &StringTensor<StrServer>::write,
            py::arg("str_list"), py::arg("index") = 0)

        .def("read_vec", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrServer>& self, 
                    std::vector<std::string>& result_vec, 
                    int index) {

            return self.read(result_vec, index);

        }, py::arg("str_list"), py::arg("index") = 0)

        .def("write_str",
            (bool (StringTensor<StrServer>::*)(const std::string&, int))
            &StringTensor<StrServer>::write,
            py::arg("to_write"), py::arg("index") = 0)

        .def("read_str", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrServer>& self, 
                    std::string& result_str, 
                    int index) {

            return self.read(result_str, index);

        }, py::arg("to_read"), py::arg("index") = 0)

        ;

}

void PySharsorIPC::PyStringTensor::declare_StringTensorClient(py::module &m) {

    std::string pyclass_name = std::string("StringTensorClient");

    py::class_<StringTensor<StrClient>>(m, pyclass_name.c_str())

        .def(py::init<std::string, std::string, bool, VLevel>(),
             py::arg("basename") = "MySharedMemory",
             py::arg("name_space") = "",
             py::arg("verbose") = false,
             py::arg("vlevel") = VLevel::V0)

        .def("run", &StringTensor<StrClient>::run)

        // Add other method bindings here ...
        .def("isRunning", &StringTensor<StrClient>::isRunning)

        .def("length", &StringTensor<StrClient>::getLength)

        .def("close", &StringTensor<StrClient>::close)

        .def("write_vec",
            (bool (StringTensor<StrClient>::*)(const std::vector<std::string>&, int))
            &StringTensor<StrClient>::write,
            py::arg("str_list"), py::arg("index") = 0)

        .def("read_vec", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrClient>& self, 
                    std::vector<std::string>& result_vec, 
                    int index) {

            return self.read(result_vec, index);

        }, py::arg("str_list"), py::arg("index") = 0)

        .def("write_str",
            (bool (StringTensor<StrClient>::*)(const std::string&, int))
            &StringTensor<StrClient>::write,
            py::arg("to_write"), py::arg("index") = 0)

        .def("read_str", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrClient>& self, 
                    std::string& result_str, 
                    int index) {

            return self.read(result_str, index);

        }, py::arg("to_read"), py::arg("index") = 0)

        ;
}
