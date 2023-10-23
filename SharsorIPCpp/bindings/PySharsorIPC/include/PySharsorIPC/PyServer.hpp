#ifndef PYSERVER_HPP
#define PYSERVER_HPP

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <SharsorIPCpp/Server.hpp>

namespace py = pybind11;
using namespace SharsorIPCpp;

enum class SrvrType {
    BOOL,
    INT,
    FLOAT,
    DOUBLE
};

// Forward declare the binding function.
void bind_Server(py::module &m);

#endif // PYSERVER_HPP
