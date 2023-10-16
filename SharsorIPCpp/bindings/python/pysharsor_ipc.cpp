#include <pybind11/pybind11.h>

#include <Server.hpp>
#include <pybind11/eigen.h>

namespace py = pybind11;

using namespace SharsorIPCpp;

PYBIND11_MODULE(PySharsorIPC, m) {

    m.doc() = "Python binding for SharsorIPCpp library";

    py::class_<Server>(m, "Server")
            .def(py::init()));

    py::class_<NumDiff, std::shared_ptr<NumDiff>>(m, "NumDiff")
    .def(py::init(construct_num_diff),
         py::arg("n_jnts"),
         py::arg("dt"),
         py::arg("order") = 1
         )
    .def("add_sample", &NumDiff::add_sample,
         py::arg("sample"))

    .def("dot", dot)
    ;

}
