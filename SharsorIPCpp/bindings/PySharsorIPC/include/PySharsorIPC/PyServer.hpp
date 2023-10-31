#ifndef PYSERVER_HPP
#define PYSERVER_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

#include <WrapUtils.hpp>
#include <PySharsorIPC/PyDTypes.hpp>

namespace py = pybind11;

using VLevel = SharsorIPCpp::Journal::VLevel;
using DType = SharsorIPCpp::DType;

namespace PySharsorIPC{

    py::object ServerFactory(int n_rows,
                             int n_cols,
                            std::string basename = "MySharedMemory",
                            std::string name_space = "",
                            bool verbose = false,
                            VLevel vlevel = VLevel::V0,
                            SharsorIPCpp::DType dtype = SharsorIPCpp::DType::Float,
                            int layout = SharsorIPCpp::ColMajor);

    void bind_ServerWrapper(py::module& m);

    template <typename Scalar,
              int Layout = SharsorIPCpp::MemLayoutDefault>
    void bindServerT(py::module &m, const char* name);

    void bindServers(py::module &m);

    void bindServerFactory(py::module &m,
                        const char* name = "ServerFactory");

}

#endif // PYSERVER_HPP
