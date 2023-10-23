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

        .def("getNRows", &SharsorIPCpp::Client<Scalar>::getNRows)

        .def("getNCols", &SharsorIPCpp::Client<Scalar>::getNCols);
}

py::object PyClient::ClientFactory(std::string basename,
                        std::string name_space,
                        bool verbose,
                        VLevel vlevel,
                        DataType dtype) {
    switch (dtype) {

        case DataType::BOOL:

            return py::cast(ClientWrapper(new py::object(
                                              py::cast(std::make_shared<Client<bool>>(basename,
                                                                                    name_space,
                                                                                    verbose,
                                                                                    vlevel)))));
        case DataType::INT:
            return py::cast(ClientWrapper(new py::object(
                                              py::cast(std::make_shared<Client<int>>(basename,
                                                                                    name_space,
                                                                                    verbose,
                                                                                    vlevel)))));
        case DataType::FLOAT:
            return py::cast(ClientWrapper(new py::object(
                                              py::cast(std::make_shared<Client<float>>(basename,
                                                                                    name_space,
                                                                                    verbose,
                                                                                    vlevel)))));
        case DataType::DOUBLE:
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

            py::tuple result = client.attr("readTensor")(row, col).cast<py::tuple>();

            bool success = result[0].cast<bool>();

            py::array tensor = result[1].cast<py::array>();

            return std::make_tuple(success, tensor);
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
          py::arg("name_space") = "",
          py::arg("verbose") = false,
          py::arg("vlevel") = VLevel::V0,
          py::arg("dtype") = DataType::FLOAT,
          "Create a new client with the specified arguments and dtype.");

}
