#include <PySharsorIPC/PyConsumer.hpp>

void PySharsorIPC::PyConsumer::bind_Consumer(pybind11::module& m) {

        pybind11::class_<SharsorIPCpp::Consumer> cls(m, "Consumer");
        
        cls.def(pybind11::init<std::string, std::string, bool, SharsorIPCpp::Journal::VLevel>(),
             pybind11::arg("basename"),
             pybind11::arg("namespace"),
             pybind11::arg("verbose"), 
             pybind11::arg("vlevel") = VLevel::V0);
        cls.def("run", &SharsorIPCpp::Consumer::run);
        cls.def("close", &SharsorIPCpp::Consumer::close);
        cls.def("wait", [](SharsorIPCpp::Consumer& self, 
                        unsigned int ms_timeout) {
            
            return self.wait(ms_timeout);

        }, pybind11::arg("ms_timeout") = -1);
        cls.def("ack", &SharsorIPCpp::Consumer::ack);

}