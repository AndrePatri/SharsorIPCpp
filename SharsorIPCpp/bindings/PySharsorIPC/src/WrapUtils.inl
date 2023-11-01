namespace PySharsorIPC {

    inline Wrapper::Wrapper(pybind11::object* Obj)
        : wrapped(std::unique_ptr<pybind11::object>(Obj)) {

    }

    template<typename Func>
    auto Wrapper::execute(Func&& f) -> decltype(f(std::declval<pybind11::object&>())) {
        return f(*wrapped);
    }

}
