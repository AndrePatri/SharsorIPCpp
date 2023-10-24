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

    // bindings of Client for Python

    py::class_<SharsorIPCpp::Client<Scalar>, std::shared_ptr<SharsorIPCpp::Client<Scalar>>>(m, name)

        .def(py::init<std::string, std::string, bool, SharsorIPCpp::VLevel>())

        .def("writeTensor", &SharsorIPCpp::Client<Scalar>::writeTensor)

        .def("readTensor", [](SharsorIPCpp::Client<Scalar>& client,
             py::array_t<Scalar>& np_array,
             int row, int col) {

            // Ensure the numpy array is mutable and get a pointer to its data
            py::buffer_info buf_info = np_array.request();

            bool success = false;

            // We check if the provided array is row-major
            if (buf_info.strides[0] > buf_info.strides[1]) {// col major

                Scalar* ptr = static_cast<Scalar*>(buf_info.ptr);

                // creates a memory map that uses the same memory as the numpy array
                MMap<Scalar> output_t(ptr,
                                      np_array.shape(0), // dimensions are taken
                                      np_array.shape(1)); // directly for the input tensor

                // Call the original readTensor method with the MMap
                success = client.readTensor(output_t, row, col); // this will
                // try to copy the data of the shared tensor into the tensor mapped by MMap

                // the numpy array should now be updated in place

            }
            else { // for now throw error. T.B. improved

                std::string error = std::string("Provided numpy array is not row-major.") +
                        std::string(" Please create the array with the order=\'C\' option to") +
                        std::string(" ensure correct reading.")
                        ;

                throw std::runtime_error(error);

            }

            return success;

        })

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

    // (here we are calling python code from C++)

    py::class_<ClientWrapper> cls(m, "Client");

    cls.def("attach", [](ClientWrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("attach")(); // calls the attach (from Client's PY bindings)

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
                             py::array& np_array,
                             int row, int col) {

        return wrapper.execute([&](py::object& client) -> bool {

            // we first get the underlying dtype of the client
            DType client_type = client.attr("getScalarType")().cast<DType>();
            // and of the input np array
            py::dtype np_dtype = np_array.dtype();

            switch (client_type) { // we check compatibility at runtime
            // (this was not necessary in C++, since it's checked at compile time)

            case DType::Bool:

                if (!np_dtype.is(py::dtype::of<bool>())) {

                    throw std::runtime_error("Mismatched dtype: "
                                             "Expected boolean numpy array.");

                }
                break;

            case DType::Int:

                if (!np_dtype.is(py::dtype::of<int>())) {

                    throw std::runtime_error("Mismatched dtype: "
                                             "Expected integer numpy array.");
                }
                break;

            case DType::Float:

                if (!np_dtype.is(py::dtype::of<float>())) {

                    throw std::runtime_error("Mismatched dtype: "
                                             "Expected float numpy array.");
                }
                break;

            case DType::Double:

                if (!np_dtype.is(py::dtype::of<double>())) {

                    throw std::runtime_error("Mismatched dtype: "
                                             "Expected double numpy array.");
                }
                break;

            default:

                throw std::runtime_error("Invalid client dtype!");

            }

            return client.attr("readTensor")(np_array,
                                             row, col).cast<bool>(); // copies in place

        });

    }, py::arg("tensor"), py::arg("row") = 0, py::arg("col") = 0);
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
