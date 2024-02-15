#include <PySharsorIPC/PyCondVar.hpp>

void PySharsorIPC::PyConditionVariable::bind_ConditionVariable(pybind11::module& m) {

        pybind11::class_<SharsorIPCpp::ConditionVariable> cls(m, "ConditionVariable");

        cls.def("wait", &SharsorIPCpp::ConditionVariable::wait);

        cls.def("notify_one", &SharsorIPCpp::ConditionVariable::notify_one);

        cls.def("notify_all", &SharsorIPCpp::ConditionVariable::notify_all);

        cls.def("close", &SharsorIPCpp::ConditionVariable::close);

}