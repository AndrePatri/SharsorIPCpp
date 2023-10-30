#ifndef PYDTYPES_HPP
#define PYDTYPES_HPP

#include <pybind11/pybind11.h>

namespace PySharsorIPC {

    template<typename Scalar>
    using NumpyArray = pybind11::array_t<Scalar>;

}

#endif // PYDTYPES_HPP
