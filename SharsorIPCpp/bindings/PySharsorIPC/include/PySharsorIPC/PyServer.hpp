#ifndef PYSERVER_HPP
#define PYSERVER_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>

#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <PySharsorIPC/PyDTypes.hpp>
#include <WrapUtils.hpp>

namespace PySharsorIPC{

    namespace PyServer {

        using VLevel = SharsorIPCpp::Journal::VLevel;
        using LogType = SharsorIPCpp::Journal::LogType;
        using DType = SharsorIPCpp::DType;

        pybind11::object ServerFactory(int n_rows,
                                int n_cols,
                                std::string basename = "MySharedMemory",
                                std::string name_space = "",
                                bool verbose = false,
                                VLevel vlevel = VLevel::V0,
                                bool force_reconnection = false,
                                SharsorIPCpp::DType dtype = SharsorIPCpp::DType::Float,
                                int layout = SharsorIPCpp::ColMajor);

        void bind_ServerWrapper(pybind11::module& m);

        template <typename Scalar,
                int Layout = SharsorIPCpp::MemLayoutDefault>
        void bindServerT(pybind11::module &m, const char* name);

        void bindServers(pybind11::module &m);

        void bindServerFactory(pybind11::module &m,
                            const char* name = "ServerFactory");

    }

}

#endif // PYSERVER_HPP
