#ifndef PYDTYPES_HPP
#define PYDTYPES_HPP

namespace PySharsorIPC {

    template<typename Scalar, int Layout>
    using NumpyArray = std::conditional_t<Layout == SharsorIPCpp::ColMajor,
                                            py::array_t<Scalar, py::array::f_style>,
                                            py::array_t<Scalar, py::array::c_style>>;

}

#endif // PYDTYPES_HPP
