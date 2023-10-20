#include <pybind11/pybind11.h>

#include <pybind11/stl.h>

#include <Eigen/Dense>

//#include <SharsorIPCpp/Server.hpp>
//#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/StringTensor.hpp>

namespace py = pybind11;
using namespace SharsorIPCpp;

template <typename ShMemType>
void declare_StringTensor(py::module &m,
                          const std::string& type) {

    std::string pyclass_name;

    if (type == std::string("Server")) {

        pyclass_name = std::string("StringTensorServer");

    }
    if (type == std::string("Client")) {

        pyclass_name = std::string("StringTensorClient");

    }

    py::class_<StringTensor<ShMemType>>(m, pyclass_name.c_str())

        .def(py::init<std::string, std::string, bool, VLevel>(),
             py::arg("basename") = "MySharedMemory",
             py::arg("name_space") = "",
             py::arg("verbose") = false,
             py::arg("vlevel") = VLevel::V0)

        .def(py::init<int, std::string, std::string, bool, VLevel, bool>(),
             py::arg("length"),
             py::arg("basename") = "MySharedMemory",
             py::arg("name_space") = "",
             py::arg("verbose") = false,
             py::arg("vlevel") = VLevel::V0,
             py::arg("force_reconnection") = false)

        .def("run", &StringTensor<ShMemType>::run)

        // Add other method bindings here ...
        .def("isRunning", &StringTensor<ShMemType>::isRunning)

        .def("close", &StringTensor<ShMemType>::close)

        .def("write",
            (bool (StringTensor<ShMemType>::*)(const std::vector<std::string>&, int))
            &StringTensor<ShMemType>::write,
            py::arg("vec"), py::arg("index") = 0)
        .def("write",
            (bool (StringTensor<ShMemType>::*)(const std::string&, int))
            &StringTensor<ShMemType>::write,
            py::arg("str"), py::arg("index") = 0)
        .def("read",
            (bool (StringTensor<ShMemType>::*)(std::vector<std::string>&, int))
            &StringTensor<ShMemType>::read,
            py::arg("vec"), py::arg("index") = 0)
        .def("read",
            (bool (StringTensor<ShMemType>::*)(std::string&, int))
            &StringTensor<ShMemType>::read,
            py::arg("str"), py::arg("index") = 0)

        ;
}

PYBIND11_MODULE(stringtensor, m) {
    m.doc() = "pybind11 StringTensor bindings";

    // Bind other classes or enums if necessary...

    declare_StringTensor<StrServer>(m, "Server");

    declare_StringTensor<StrClient>(m, "Client");
}

