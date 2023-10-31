#include <WrapUtils.hpp>

PySharsorIPC::ClientWrapper::ClientWrapper(pybind11::object* Obj) :

    wrapped(std::unique_ptr<pybind11::object>(Obj)) {

}

template<typename Func>
auto PySharsorIPC::ClientWrapper::execute(Func&& f) -> decltype(f(std::declval<pybind11::object&>())) {

    return f(*wrapped);

}

PySharsorIPC::ServerWrapper::ServerWrapper(pybind11::object* Obj) :

    wrapped(std::unique_ptr<pybind11::object>(Obj)) {

}

template<typename Func>
auto PySharsorIPC::ServerWrapper::execute(Func&& f) -> decltype(f(std::declval<pybind11::object&>())) {

    return f(*wrapped);

}
