#include <PySharsorIPC/PyClient.hpp>
#include <PyDTypes.hpp>

PySharsorIPC::PyClient::ClientWrapper::ClientWrapper(py::object* clientObj) :

    client(std::unique_ptr<py::object>(clientObj)) {

}

template<typename Func>
auto PySharsorIPC::PyClient::ClientWrapper::execute(Func&& f) -> decltype(f(std::declval<py::object&>())) {

    return f(*client);

}

template <typename Scalar, int Layout>
void PySharsorIPC::PyClient::bindClientT(py::module &m, const char* name) {

    // bindings of Client for Python

    py::class_<SharsorIPCpp::Client<Scalar, Layout>,
            std::shared_ptr<SharsorIPCpp::Client<Scalar, Layout>>>(m, name)

        .def(py::init<std::string, std::string, bool, SharsorIPCpp::VLevel>())

        .def("writeTensor", &SharsorIPCpp::Client<Scalar, Layout>::writeTensor)

        .def("readTensor", [](SharsorIPCpp::Client<Scalar, Layout>& client,
             NumpyArray<Scalar, Layout>& np_array,
             int row, int col) {

            // Ensure the numpy array is mutable and get a pointer to its data
            py::buffer_info buf_info = np_array.request();

            bool success = false;

            Scalar* start_ptr = static_cast<Scalar*>(buf_info.ptr);

            bool is_columnmajor = np_array.flags() & py::array::f_style;

            bool is_colmaj_coherent = (is_columnmajor && (Layout == SharsorIPCpp::ColMajor));
            bool is_rowmaj_coherent = (!is_columnmajor && (Layout == SharsorIPCpp::RowMajor));

            if (is_colmaj_coherent) // col. major client layout
            { // array and client are consistent

                // Define the stride based on the layout of the matrix
                SharsorIPCpp::DStrides strides(np_array.shape(0),
                                1);

                // creates a (dynamic) memory map that uses the same memory
                // and layout as the numpy array
                SharsorIPCpp::TensorView<Scalar, Layout> output_t(start_ptr,
                              np_array.shape(0), // dimensions are taken
                              np_array.shape(1),
                              strides); // directly for the input tensor

                // Call the original readTensor method with the TensorView
                success = client.readTensor(output_t,
                                            row, col); // this will
                // try to copy the data of the shared tensor into the
                // tensor mapped by TensorView

            }
            if (is_rowmaj_coherent)
            { // array and client are consistent

                SharsorIPCpp::DStrides strides(np_array.shape(1),
                                               1);

                SharsorIPCpp::TensorView<Scalar, Layout> output_t(start_ptr,
                              np_array.shape(0),
                              np_array.shape(1),
                              strides);

                success = client.readTensor(output_t,
                                            row, col);

            }
//            if (!is_colmaj_coherent && !is_rowmaj_coherent) { // not coherent


//            }

            return success;

        })

        .def("attach", &SharsorIPCpp::Client<Scalar, Layout>::attach)

        .def("detach", &SharsorIPCpp::Client<Scalar, Layout>::detach)

        .def("close", &SharsorIPCpp::Client<Scalar, Layout>::close)

        .def("isAttached", &SharsorIPCpp::Client<Scalar, Layout>::isAttached)

        .def("getScalarType", &SharsorIPCpp::Client<Scalar, Layout>::getScalarType)

        .def("getNRows", &SharsorIPCpp::Client<Scalar, Layout>::getNRows)

        .def("getNCols", &SharsorIPCpp::Client<Scalar, Layout>::getNCols);
}

py::object PySharsorIPC::PyClient::ClientFactory(std::string basename,
                        std::string name_space,
                        bool verbose,
                        VLevel vlevel,
                        DType dtype,
                        int layout) {

    switch (layout) {

        case SharsorIPCpp::ColMajor:

            switch (dtype) {

                case DType::Bool:

                    return py::cast(ClientWrapper(new py::object(
                                          py::cast(std::make_shared<SharsorIPCpp::Client<bool, SharsorIPCpp::ColMajor>>(basename,
                                                                                name_space,
                                                                                verbose,
                                                                                vlevel)))));
                case DType::Int:

                    return py::cast(ClientWrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<int, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Float:

                    return py::cast(ClientWrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<float, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Double:

                    return py::cast(ClientWrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<double, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                default:

                    throw std::runtime_error("Invalid dtype specified!");
            }

        case SharsorIPCpp::RowMajor:

            switch (dtype) {

                case DType::Bool:

                    return py::cast(ClientWrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<bool, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Int:
                    return py::cast(ClientWrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<int, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Float:
                    return py::cast(ClientWrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<float, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Double:
                    return py::cast(ClientWrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<double, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                default:

                    throw std::runtime_error("Invalid dtype specified!");
            }

        default:

            throw std::runtime_error("Invalid layout specified!");

    }
}

void PySharsorIPC::PyClient::bind_ClientWrapper(py::module& m) {

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

void PySharsorIPC::PyClient::bindClients(py::module& m) {

    bindClientT<bool, SharsorIPCpp::ColMajor>(m, "__PyClientBoolColMaj");
    bindClientT<bool, SharsorIPCpp::RowMajor>(m, "__PyClientBoolRowMaj");

    bindClientT<int, SharsorIPCpp::ColMajor>(m, "__PyClientIntColMaj");
    bindClientT<int, SharsorIPCpp::RowMajor>(m, "__PyClientIntRowMaj");

    bindClientT<float, SharsorIPCpp::ColMajor>(m, "__PyClientFloatColMaj");
    bindClientT<float, SharsorIPCpp::RowMajor>(m, "__PyClientFloatRowMaj");

    bindClientT<double, SharsorIPCpp::ColMajor>(m, "__PyClientDoubleColMaj");
    bindClientT<double, SharsorIPCpp::RowMajor>(m, "__PyClientDoubleRowMaj");

}

void PySharsorIPC::PyClient::bindFactory(py::module& m,
                           const char* name) {

    m.def(name, &PyClient::ClientFactory,
          py::arg("basename") = "MySharedMemory",
          py::arg("namespace") = "",
          py::arg("verbose") = false,
          py::arg("vlevel") = VLevel::V0,
          py::arg("dtype") = DType::Float,
          py::arg("layout") = SharsorIPCpp::ColMajor,
          "Create a new client with the specified arguments and dtype.");

}
