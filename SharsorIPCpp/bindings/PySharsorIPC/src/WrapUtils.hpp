#ifndef WRAPUTILS_HPP
#define WRAPUTILS_HPP

#include <pybind11/pybind11.h>
#include <memory>
#include <type_traits>

namespace PySharsorIPC {

    class Wrapper {

        public:

            Wrapper(pybind11::object* Obj);

            template<typename Func>
            auto execute(Func&& f) -> decltype(f(std::declval<pybind11::object&>()));

        public:

            std::unique_ptr<pybind11::object> wrapped;

    };

}

#include "WrapUtils.inl"

#endif  // WRAPUTILS_HPP

