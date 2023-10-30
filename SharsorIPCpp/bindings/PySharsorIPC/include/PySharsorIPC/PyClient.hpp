#ifndef PYCLIENT_HPP
#define PYCLIENT_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>

#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

#include <WrapUtils.hpp>
#include <PySharsorIPC/PyDTypes.hpp>

namespace py = pybind11;

using VLevel = SharsorIPCpp::Journal::VLevel;
using DType = SharsorIPCpp::DType;

namespace PySharsorIPC{

    py::object ClientFactory(std::string basename = "MySharedMemory",
                            std::string name_space = "",
                            bool verbose = false,
                            VLevel vlevel = VLevel::V0,
                            SharsorIPCpp::DType dtype = SharsorIPCpp::DType::Float,
                            int layout = SharsorIPCpp::ColMajor);

    void bind_ClientWrapper(py::module& m);

    template <typename Scalar,
              int Layout = SharsorIPCpp::MemLayoutDefault>
    void bindClientT(py::module &m, const char* name);

    void bindClients(py::module &m);

    void bindClientFactory(py::module &m,
                           const char* name = "ClientFactory");

}


#endif // PYCLIENT_HPP
