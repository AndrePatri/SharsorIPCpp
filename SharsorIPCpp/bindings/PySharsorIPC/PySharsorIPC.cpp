#include <pybind11/pybind11.h>

#include "PyStringTensor.hpp"

namespace py = pybind11;

namespace py = pybind11;
using namespace SharsorIPCpp;

PYBIND11_MODULE(PySharsorIPC, m) {

    m.doc() = "pybind11 StringTensor bindings";

    py::enum_<VLevel>(m, "VLevel")
        .value("V0", Journal::VLevel::V0)
        .value("V1", Journal::VLevel::V1)
        .value("V2", Journal::VLevel::V2)
        .value("V3", Journal::VLevel::V3)
        .export_values();

    declare_StringTensorServer(m);

    declare_StringTensorClient(m);

}

