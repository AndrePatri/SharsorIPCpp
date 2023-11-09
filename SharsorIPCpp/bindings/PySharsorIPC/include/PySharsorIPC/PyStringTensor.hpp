#ifndef PYSTRINGTENSOR_HPP
#define PYSTRINGTENSOR_HPP

#include <pybind11/pybind11.h>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/StringTensor.hpp>

namespace py = pybind11;

namespace PySharsorIPC {

    namespace PyStringTensor{
        
        using namespace SharsorIPCpp;
 
        using VLevel = Journal::VLevel;
        using LogType = Journal::LogType;
        
        void declare_StringTensorServer(py::module &m);
        void declare_StringTensorClient(py::module &m);

    }

}

#endif // PYSTRINGTENSOR_HPP
