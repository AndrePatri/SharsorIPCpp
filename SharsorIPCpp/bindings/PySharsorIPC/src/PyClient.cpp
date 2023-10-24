#include <PySharsorIPC/PyClient.hpp>

PyClient::ClientWrapper::ClientWrapper(py::object* clientObj) :

    client(std::unique_ptr<py::object>(clientObj)) {

}

template<typename Func>
auto PyClient::ClientWrapper::execute(Func&& f) -> decltype(f(std::declval<py::object&>())) {

    return f(*client);

}

template <typename Scalar>
void PyClient::bindClientT(py::module &m, const char* name) {

    py::class_<SharsorIPCpp::Client<Scalar>, std::shared_ptr<SharsorIPCpp::Client<Scalar>>>(m, name)

        .def(py::init<std::string, std::string, bool, SharsorIPCpp::VLevel>())

        .def("writeTensor", &SharsorIPCpp::Client<Scalar>::writeTensor)

        .def("readTensor", &SharsorIPCpp::Client<Scalar>::readTensor)

        .def("attach", &SharsorIPCpp::Client<Scalar>::attach)

        .def("detach", &SharsorIPCpp::Client<Scalar>::detach)

        .def("close", &SharsorIPCpp::Client<Scalar>::close)

        .def("isAttached", &SharsorIPCpp::Client<Scalar>::isAttached)

        .def("getScalarType", &SharsorIPCpp::Client<Scalar>::getScalarType)

        .def("getNRows", &SharsorIPCpp::Client<Scalar>::getNRows)

        .def("getNCols", &SharsorIPCpp::Client<Scalar>::getNCols);
}

py::object PyClient::ClientFactory(std::string basename,
                        std::string name_space,
                        bool verbose,
                        VLevel vlevel,
                        DType dtype) {
    switch (dtype) {

        case DType::Bool:

            return py::cast(ClientWrapper(new py::object(
                                              py::cast(std::make_shared<Client<bool>>(basename,
                                                                                    name_space,
                                                                                    verbose,
                                                                                    vlevel)))));
        case DType::Int:
            return py::cast(ClientWrapper(new py::object(
                                              py::cast(std::make_shared<Client<int>>(basename,
                                                                                    name_space,
                                                                                    verbose,
                                                                                    vlevel)))));
        case DType::Float:
            return py::cast(ClientWrapper(new py::object(
                                              py::cast(std::make_shared<Client<float>>(basename,
                                                                                    name_space,
                                                                                    verbose,
                                                                                    vlevel)))));
        case DType::Double:
            return py::cast(ClientWrapper(new py::object(
                                              py::cast(std::make_shared<Client<double>>(basename,
                                                                                    name_space,
                                                                                    verbose,
                                                                                    vlevel)))));
        default:

            throw std::runtime_error("Invalid dtype specified!");
    }
}

void PyClient::bind_ClientWrapper(py::module& m) {

    // By handling everything in Python-space,
    // we eliminate the need to handle the type
    // conversion in C++, making the ClientWrapper
    // agnostic to the actual data type of the underlying Client.

    py::class_<ClientWrapper> cls(m, "Client");

    cls.def("attach", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("attach")();

        });

    });

    cls.def("detach", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("detach")();

        });

    });

    cls.def("close", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("close")();

        });

    });

    cls.def("isAttached", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("isAttached")().cast<bool>();

        });

    });

    cls.def("getNRows", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("getNRows")().cast<int>();

        });

    });

    cls.def("getNCols", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("getNCols")().cast<int>();

        });

    });

    cls.def("getScalarType", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("getScalarType")().cast<DType>();

        });

    });

    cls.def("writeTensor", [](ClientWrapper& wrapper,
                              const py::array& np_array,
                              int row, int col) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("writeTensor")(np_array, row, col);

        });

    }, py::arg("data"), py::arg("row") = 0, py::arg("col") = 0);

    cls.def("readTensor", [](ClientWrapper& wrapper,
                             int row, int col) {

        return wrapper.execute([&](py::object& client) {

            DType client_type = client.attr("getScalarType")().cast<DType>();

            int nRows = client.attr("getNRows")().cast<int>();
            int nCols = client.attr("getNCols")().cast<int>();

            bool is_attached = client.attr("isAttached")().cast<bool>();

            bool success = false;

            switch (client_type) {

                case DType::Bool:
                    {
                        if(is_attached) {

                            Tensor<bool> output_t(nRows, nCols);

                            success = client.attr("readTensor")(output_t, row, col).cast<bool>();

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor);
                        }
                        else {

                            Tensor<bool> output_t(1, 1); // 1x1 default

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor); // success is false
                        }
                    }
                    break;

                case DType::Int:
                    {
                        if(is_attached) {

                            Tensor<int> output_t(nRows, nCols);

                            success = client.attr("readTensor")(output_t, row, col).cast<bool>();

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor);
                        }
                        else {

                            Tensor<int> output_t(1, 1); // 1x1 default

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor); // success is false
                        }
                    }
                    break;

                case DType::Float:
                    {
                        if(is_attached) {

                            Tensor<float> output_t(nRows, nCols);

                            success = client.attr("readTensor")(output_t, row, col).cast<bool>();

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor);
                        }
                        else {

                            Tensor<float> output_t(1, 1); // 1x1 default

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor); // success is false
                        }
                    }
                    break;

                case DType::Double:
                    {
                        if(is_attached) {

                            Tensor<double> output_t(nRows, nCols);

                            success = client.attr("readTensor")(output_t, row, col).cast<bool>();

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor);
                        }
                        else {

                            Tensor<double> output_t(1, 1); // 1x1 default

                            py::array tensor = pybind11::cast(output_t);

                            return std::make_tuple(success, tensor); // success is false
                        }
                    }
                    break;

                default:

                    throw std::runtime_error("Invalid dtype!");
            }

        });

    }, py::arg("row") = 0, py::arg("col") = 0);
}

void PyClient::bindClients(py::module& m) {

    bindClientT<bool>(m, "__PyClientBool");

    bindClientT<int>(m, "__PyClientInt");

    bindClientT<float>(m, "__PyClientFloat");

    bindClientT<double>(m, "__PyClientDouble");
}

void PyClient::bindFactory(py::module& m,
                           const char* name) {

    m.def(name, &PyClient::ClientFactory,
          py::arg("basename") = "MySharedMemory",
          py::arg("namespace") = "",
          py::arg("verbose") = false,
          py::arg("vlevel") = VLevel::V0,
          py::arg("dtype") = DType::Float,
          "Create a new client with the specified arguments and dtype.");

}
