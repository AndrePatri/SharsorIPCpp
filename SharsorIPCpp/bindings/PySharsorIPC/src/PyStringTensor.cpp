#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <pybind11/numpy.h>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Server.hpp>

#include <PySharsorIPC/PyStringTensor.hpp>

namespace py = pybind11;
using namespace SharsorIPCpp;

void declare_StringTensorServer(py::module &m) {

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

        .def("write",
            (bool (StringTensor<StrServer>::*)(const std::vector<std::string>&, int))
            &StringTensor<StrServer>::write,
            py::arg("vec"), py::arg("index") = 0)

        .def("write",
            (bool (StringTensor<StrServer>::*)(const std::string&, int))
            &StringTensor<StrServer>::write,
            py::arg("str"), py::arg("index") = 0)

        .def("read_str", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrServer>& self, int index) {

                    std::string result_str;

                    bool success = false;

                    if (self.isRunning()) {

                        success = self.read(result_str, index);
                    }

                    return std::make_tuple(success, result_str);

                }, py::arg("index") = 0)

        .def("read_vec", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrServer>& self, int index) {

            std::vector<std::string> result_vec;

            bool success = false;

            if (self.isRunning()) {

                result_vec.resize(self.getLength());

                success = self.read(result_vec, index);

            }

            return std::make_tuple(success, result_vec);

        }, py::arg("index") = 0)

        ;

}

void declare_StringTensorClient(py::module &m) {

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

        .def("write",
            (bool (StringTensor<StrClient>::*)(const std::vector<std::string>&, int))
            &StringTensor<StrClient>::write,
            py::arg("vec"), py::arg("index") = 0)
        .def("write",
            (bool (StringTensor<StrClient>::*)(const std::string&, int))
            &StringTensor<StrClient>::write,
            py::arg("str"), py::arg("index") = 0)

        .def("read_str", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrClient>& self, int index) {

                    std::string result_str;

                    bool success = false;

                    if (self.isRunning()) {

                        success = self.read(result_str, index);
                    }

                    return std::make_tuple(success, result_str);

                }, py::arg("index") = 0)

        .def("read_vec", [](SharsorIPCpp::StringTensor<SharsorIPCpp::StrClient>& self, int index) {

            std::vector<std::string> result_vec;

            bool success = false;

            if (self.isRunning()) {

                result_vec.resize(self.getLength());

                success = self.read(result_vec, index);

            }

            return std::make_tuple(success, result_vec);

        }, py::arg("index") = 0)

        ;

}
