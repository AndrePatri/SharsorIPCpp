#include <pybind11/pybind11.h>

#include <PySharsorIPC/PyStringTensor.hpp>
#include <PySharsorIPC/PyServer.hpp>
#include <PySharsorIPC/PyClient.hpp>


namespace py = pybind11;

namespace py = pybind11;
using namespace SharsorIPCpp;

PYBIND11_MODULE(PySharsorIPC, m) {

    m.doc() = "pybind11 StringTensor bindings";

    py::enum_<DataType>(m, "dtype")
        .value("BOOL", DataType::BOOL)
        .value("INT", DataType::INT)
        .value("FLOAT", DataType::FLOAT)
        .value("DOUBLE", DataType::DOUBLE);

    py::enum_<VLevel>(m, "VLevel")
        .value("V0", Journal::VLevel::V0)
        .value("V1", Journal::VLevel::V1)
        .value("V2", Journal::VLevel::V2)
        .value("V3", Journal::VLevel::V3)
        .export_values();

//    bind_Server(m);

    // Client bindings
    PyClient::bindClients(m); // binds all client types

    PyClient::bind_ClientWrapper(m); // binds the client wrapper

    PyClient::bindFactory(m); // binds the factory (runtime Client creation)

    // String tensor bindings
    declare_StringTensorServer(m);

    declare_StringTensorClient(m);



}

