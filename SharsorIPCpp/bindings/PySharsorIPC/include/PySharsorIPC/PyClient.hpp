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

using VLevel = SharsorIPCpp::Journal::VLevel;
using DType = SharsorIPCpp::DType;

namespace PySharsorIPC{

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


#endif // PYCLIENT_HPP
