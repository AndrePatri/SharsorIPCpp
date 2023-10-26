#ifndef PYCLIENT_HPP
#define PYCLIENT_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>

#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

namespace py = pybind11;

using VLevel = SharsorIPCpp::Journal::VLevel;
using DType = SharsorIPCpp::DType;

namespace PySharsorIPC::PyClient {

        class ClientWrapper {

            public:

                ClientWrapper(py::object* clientObj);

                template<typename Func>
                auto execute(Func&& f) -> decltype(f(std::declval<py::object&>()));

            private:

                std::unique_ptr<py::object> client;

        };

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

        void bindFactory(py::module &m, const char* name = "ClientFactory");

}


#endif // PYCLIENT_HPP
