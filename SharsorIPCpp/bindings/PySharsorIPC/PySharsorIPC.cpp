#include <pybind11/pybind11.h>

#include <PySharsorIPC/PyStringTensor.hpp>
#include <PySharsorIPC/PyServer.hpp>
#include <PySharsorIPC/PyClient.hpp>


namespace py = pybind11;

using namespace SharsorIPCpp;
using namespace PySharsorIPC;

PYBIND11_MODULE(PySharsorIPC, m) {

    m.doc() = "pybind11 StringTensor bindings";

    py::enum_<DType>(m, "dtype")
        .value("Bool", DType::Bool)
        .value("Int", DType::Int)
        .value("Float", DType::Float)
        .value("Double", DType::Double);

    m.attr("RowMajor") = RowMajor;
    m.attr("ColMajor") = ColMajor;

    py::enum_<VLevel>(m, "VLevel")
        .value("V0", Journal::VLevel::V0)
        .value("V1", Journal::VLevel::V1)
        .value("V2", Journal::VLevel::V2)
        .value("V3", Journal::VLevel::V3)
        .export_values();

    // In your Pybind11 bindings:
    m.def("toNumpyDType", [](DType dtype) {
        switch(dtype) {
            case DType::Bool: return py::dtype::of<bool>();
            case DType::Int: return py::dtype::of<int>();
            case DType::Float: return py::dtype::of<float>();
            case DType::Double: return py::dtype::of<double>();

            default: throw std::runtime_error("Unsupported DType conversion!");
        }
    });

    // Client bindings
    bindClients(m); // binds all client types

    bind_ClientWrapper(m); // binds the client wrapper

    bindClientFactory(m); // binds the factory for Clients

    // Server bindings

    // String tensor bindings
    declare_StringTensorServer(m);

    declare_StringTensorClient(m);

}

