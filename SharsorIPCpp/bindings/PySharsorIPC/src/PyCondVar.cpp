#include <PySharsorIPC/PyCondVar.hpp>

void PySharsorIPC::PyConditionVariable::bind_ConditionVariable(pybind11::module& m) {

        pybind11::class_<PySharsorIPC::ConditionVariable> cls(m, "ConditionVariable");

        .def("wait", &SharsorIPCpp::ConditionVariable::wait);

        .def("notify_one", &SharsorIPCpp::ConditionVariable::notify_one);

        .def("notify_all", &SharsorIPCpp::ConditionVariable::notify_all);

        .def("close", &SharsorIPCpp::ConditionVariable::close);

}


