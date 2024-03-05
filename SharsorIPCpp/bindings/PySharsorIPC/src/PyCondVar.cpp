#include <PySharsorIPC/PyCondVar.hpp>

void PySharsorIPC::PyConditionVariable::bind_ConditionVariable(pybind11::module& m) {

        pybind11::class_<SharsorIPCpp::ConditionVariable> cls(m, "ConditionVariable");
        
        cls.def(pybind11::init<bool, std::string, std::string, bool, SharsorIPCpp::Journal::VLevel>());
        cls.def("wait", &SharsorIPCpp::ConditionVariable::wait);
        cls.def("wait_for", &SharsorIPCpp::ConditionVariable::wait_for);
        cls.def("timedwait", &SharsorIPCpp::ConditionVariable::timedwait);
        cls.def("timedwait_for", &SharsorIPCpp::ConditionVariable::timedwait_for);
        cls.def("notify_one", &SharsorIPCpp::ConditionVariable::notify_one);
        cls.def("notify_all", &SharsorIPCpp::ConditionVariable::notify_all);
        cls.def("close", &SharsorIPCpp::ConditionVariable::close);
        
}