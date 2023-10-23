#ifndef PYCLIENT_HPP
#define PYCLIENT_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>

#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>

namespace py = pybind11;

using namespace SharsorIPCpp;

using VLevel = Journal::VLevel;

namespace PyClient{

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
                            DType dtype = DType::Float);

    void bind_ClientWrapper(py::module& m);

    template <typename Scalar>
    void bindClientT(py::module &m, const char* name);

    void bindClients(py::module &m);

    void bindFactory(py::module &m, const char* name = "ClientFactory");

}


#endif // PYCLIENT_HPP
