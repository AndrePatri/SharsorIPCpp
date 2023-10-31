#include <PySharsorIPC/PyClient.hpp>
#include <SharsorIPCpp/Journal.hpp>

template <typename Scalar, int Layout>
void PySharsorIPC::bindClientT(py::module &m, const char* name) {

    // bindings of Client for Python

    py::class_<SharsorIPCpp::Client<Scalar, Layout>,
            std::shared_ptr<SharsorIPCpp::Client<Scalar, Layout>>>(m, name)

        .def(py::init<std::string, std::string, bool, SharsorIPCpp::VLevel>())

        .def("readTensor", [](SharsorIPCpp::Client<Scalar, Layout>& self,
                       PySharsorIPC::NumpyArray<Scalar>& arr,
                       int row, int col) {

            // runtime argument type checks are run the Wrapper

            // we get the strides directly from the array
            // (since we use the strides, we don't care if it's rowmajor
            // or colmajor, the readTensor will handle it smoothly)

            auto np_strides = arr.strides();

            SharsorIPCpp::DStrides strides(np_strides[0] / sizeof(Scalar),
                                           np_strides[1] / sizeof(Scalar));

            // Ensure the numpy array is mutable and get a pointer to its data
            py::buffer_info buf_info = arr.request();

            bool success = false;

            Scalar* start_ptr = static_cast<Scalar*>(buf_info.ptr);
            SharsorIPCpp::TensorView<Scalar, Layout> output_t(start_ptr,
                                          arr.shape(0),
                                          arr.shape(1),
                                          strides);

            success = self.readTensor(output_t,
                                      row, col);

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

py::object PySharsorIPC::ClientFactory(std::string basename,
                        std::string name_space,
                        bool verbose,
                        VLevel vlevel,
                        DType dtype,
                        int layout) {

    switch (layout) {

        case SharsorIPCpp::ColMajor:

            switch (dtype) {

                case DType::Bool:

                    return py::cast(PySharsorIPC::Wrapper(new py::object(
                                          py::cast(std::make_shared<SharsorIPCpp::Client<bool, SharsorIPCpp::ColMajor>>(basename,
                                                                                name_space,
                                                                                verbose,
                                                                                vlevel)))));
                case DType::Int:

                    return py::cast(PySharsorIPC::Wrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<int, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Float:

                    return py::cast(PySharsorIPC::Wrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<float, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Double:

                    return py::cast(PySharsorIPC::Wrapper(new py::object(
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

                    return py::cast(PySharsorIPC::Wrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<bool, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Int:
                    return py::cast(PySharsorIPC::Wrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<int, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Float:
                    return py::cast(PySharsorIPC::Wrapper(new py::object(
                                                      py::cast(std::make_shared<SharsorIPCpp::Client<float, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Double:
                    return py::cast(PySharsorIPC::Wrapper(new py::object(
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

void PySharsorIPC::bind_ClientWrapper(py::module& m) {

    // By handling everything in Python-space,
    // we eliminate the need to handle the type
    // conversion in C++, making the ClientWrapper
    // agnostic to the actual data type of the underlying Client.

    // Here we lose a bit of performance wrt using the bare class bindings,
    // since additional logic and checks are necessary. However, we gain
    // much cleaner interface and flexibility

    // (here we are calling python code from C++)

    py::class_<PySharsorIPC::Wrapper> cls(m, "Client");

    cls.def("attach", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("attach")(); // calls the attach (from Client's PY bindings)

        });

    });

    cls.def("detach", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("detach")();

        });

    });

    cls.def("close", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("close")();

        });

    });

    cls.def("isAttached", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("isAttached")().cast<bool>();

        });

    });

    cls.def("getNRows", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("getNRows")().cast<int>();

        });

    });

    cls.def("getNCols", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("getNCols")().cast<int>();

        });

    });

    cls.def("getScalarType", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("getScalarType")().cast<DType>();

        });

    });

    cls.def("writeTensor", [](PySharsorIPC::Wrapper& wrapper,
                              const py::array& np_array,
                              int row, int col) {

        return wrapper.execute([&](py::object& client) {

            return client.attr("writeTensor")(np_array, row, col);

        });

    }, py::arg("data"), py::arg("row") = 0, py::arg("col") = 0);

    cls.def("readTensor", [](PySharsorIPC::Wrapper& wrapper,
                             py::array& np_array,
                             int row, int col) {

        return wrapper.execute([&](py::object& client) -> bool {

            // we need to check data compatibility at runtime
            // (this was not necessary in C++, since it's something
            // checked at compile time)
            // this introduces a slight overhead, but avoids unpredicted
            // behavior when the user passes non-compatible types

            // and of the input np array
            py::dtype np_dtype = np_array.dtype();

            switch (client.attr("getScalarType")().cast<DType>()) {

            case DType::Bool:

                if (!np_dtype.is(py::dtype::of<bool>())) {

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 "Mismatched dtype: expected boolean numpy array.",
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);

                }

                break;

            case DType::Int:

                if (!np_dtype.is(py::dtype::of<int>())) {

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 "Mismatched dtype: expected integer numpy array.",
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            case DType::Float:

                if (!np_dtype.is(py::dtype::of<float>())) {

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 "Mismatched dtype: expected float numpy array.",
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            case DType::Double:

                if (!np_dtype.is(py::dtype::of<double>())) {

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 "Mismatched dtype: expected double numpy array.",
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            default:

                SharsorIPCpp::Journal::log("Client",
                             "readTensor",
                             "Mismatched dtype: provided dtype not supported.",
                             SharsorIPCpp::LogType::EXCEP,
                             true);

            }

            // now we can safely read the tensor
            return client.attr("readTensor")(np_array,
                                             row, col).cast<bool>();

        });

    }, py::arg("tensor"), py::arg("row") = 0, py::arg("col") = 0);
}

void PySharsorIPC::bindClients(py::module& m) {

    bindClientT<bool, SharsorIPCpp::ColMajor>(m, "PyClientBoolColMaj");
    bindClientT<bool, SharsorIPCpp::RowMajor>(m, "PyClientBoolRowMaj");

    bindClientT<int, SharsorIPCpp::ColMajor>(m, "PyClientIntColMaj");
    bindClientT<int, SharsorIPCpp::RowMajor>(m, "PyClientIntRowMaj");

    bindClientT<float, SharsorIPCpp::ColMajor>(m, "PyClientFloatColMaj");
    bindClientT<float, SharsorIPCpp::RowMajor>(m, "PyClientFloatRowMaj");

    bindClientT<double, SharsorIPCpp::ColMajor>(m, "PyClientDoubleColMaj");
    bindClientT<double, SharsorIPCpp::RowMajor>(m, "PyClientDoubleRowMaj");

}

void PySharsorIPC::bindClientFactory(py::module& m,
                           const char* name) {

    m.def(name, &ClientFactory,
          py::arg("basename") = "MySharedMemory",
          py::arg("namespace") = "",
          py::arg("verbose") = false,
          py::arg("vlevel") = VLevel::V0,
          py::arg("dtype") = DType::Float,
          py::arg("layout") = SharsorIPCpp::RowMajor, // default of numpy and pytorch
          "Create a new client with the specified arguments and dtype.");

}
