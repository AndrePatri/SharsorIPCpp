#include <PySharsorIPC/PyClient.hpp>
#include <SharsorIPCpp/Journal.hpp>

template <typename Scalar, int Layout>
void PySharsorIPC::bindClientT(pybind11::module &m, const char* name) {

    // bindings of Client for Python

    pybind11::class_<SharsorIPCpp::Client<Scalar, Layout>,
            std::shared_ptr<SharsorIPCpp::Client<Scalar, Layout>>>(m, name)

        .def(pybind11::init<std::string, std::string, bool, SharsorIPCpp::VLevel>())

        .def("writeTensor", [](SharsorIPCpp::Client<Scalar, Layout>& self,
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
            pybind11::buffer_info buf_info = arr.request();

            bool success = false;

            Scalar* start_ptr = static_cast<Scalar*>(buf_info.ptr);
            SharsorIPCpp::TensorView<Scalar, Layout> output_t(start_ptr,
                                          arr.shape(0),
                                          arr.shape(1),
                                          strides);

            success = self.writeTensor(output_t,
                                      row, col);

            return success;

        })

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
            pybind11::buffer_info buf_info = arr.request();

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

pybind11::object PySharsorIPC::ClientFactory(std::string basename,
                        std::string name_space,
                        bool verbose,
                        VLevel vlevel,
                        DType dtype,
                        int layout) {

    switch (layout) {

        case SharsorIPCpp::ColMajor:

            switch (dtype) {

                case DType::Bool:

                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                          pybind11::cast(std::make_shared<SharsorIPCpp::Client<bool, SharsorIPCpp::ColMajor>>(basename,
                                                                                name_space,
                                                                                verbose,
                                                                                vlevel)))));
                case DType::Int:

                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Client<int, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Float:

                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Client<float, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Double:

                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Client<double, SharsorIPCpp::ColMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                default:

                    throw std::runtime_error("Invalid dtype specified!");
            }

        case SharsorIPCpp::RowMajor:

            switch (dtype) {

                case DType::Bool:

                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Client<bool, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Int:
                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Client<int, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Float:
                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Client<float, SharsorIPCpp::RowMajor>>(basename,
                                                                                            name_space,
                                                                                            verbose,
                                                                                            vlevel)))));
                case DType::Double:
                    return pybind11::cast(PySharsorIPC::Wrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Client<double, SharsorIPCpp::RowMajor>>(basename,
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

void PySharsorIPC::bind_ClientWrapper(pybind11::module& m) {

    // By handling everything in Python-space,
    // we eliminate the need to handle the type
    // conversion in C++, making the ClientWrapper
    // agnostic to the actual data type of the underlying Client.

    // Here we lose a bit of performance wrt using the bare class bindings,
    // since additional logic and checks are necessary. However, we gain
    // much cleaner interface and flexibility

    // (here we are calling python code from C++)

    pybind11::class_<PySharsorIPC::Wrapper> cls(m, "Client");

    cls.def("attach", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& client) {

            return client.attr("attach")(); // calls the attach (from Client's PY bindings)

        });

    });

    cls.def("detach", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& client) {

            return client.attr("detach")();

        });

    });

    cls.def("close", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& client) {

            return client.attr("close")();

        });

    });

    cls.def("isAttached", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& client) {

            return client.attr("isAttached")().cast<bool>();

        });

    });

    cls.def("getNRows", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& client) {

            return client.attr("getNRows")().cast<int>();

        });

    });

    cls.def("getNCols", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& client) {

            return client.attr("getNCols")().cast<int>();

        });

    });

    cls.def("getScalarType", [](PySharsorIPC::Wrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& client) {

            return client.attr("getScalarType")().cast<DType>();

        });

    });

    cls.def("writeTensor", [](PySharsorIPC::Wrapper& wrapper,
                             pybind11::array& np_array,
                             int row, int col) {

        return wrapper.execute([&](pybind11::object& server) -> bool {

            // we need to check data compatibility at runtime
            // (this was not necessary in C++, since it's something
            // checked at compile time)
            // this introduces a slight overhead, but avoids unpredicted
            // behavior when the user passes non-compatible types

            // and of the input np array
            pybind11::dtype np_dtype = np_array.dtype();

            switch (server.attr("getScalarType")().cast<DType>()) {

            case DType::Bool:

                if (!np_dtype.is(pybind11::dtype::of<bool>())) {

                    std::string error = std::string("Mismatched dtype: expected bool numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Server",
                                 "writeTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);

                }

                break;

            case DType::Int:

                if (!np_dtype.is(pybind11::dtype::of<int>())) {

                    std::string error = std::string("Mismatched dtype: expected int numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Server",
                                 "writeTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            case DType::Float:

                if (!np_dtype.is(pybind11::dtype::of<float>())) {

                    std::string error = std::string("Mismatched dtype: expected float numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Server",
                                 "writeTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            case DType::Double:

                if (!np_dtype.is(pybind11::dtype::of<double>())) {

                    std::string error = std::string("Mismatched dtype: expected double numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Server",
                                 "writeTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            default:

                SharsorIPCpp::Journal::log("Server",
                             "writeTensor",
                             "Mismatched dtype: provided dtype not supported.",
                             SharsorIPCpp::LogType::EXCEP,
                             true);

            }

            // now we can safely read the tensor
            return server.attr("writeTensor")(np_array,
                                             row, col).cast<bool>();

        });

    }, pybind11::arg("data"), pybind11::arg("row") = 0, pybind11::arg("col") = 0);

    cls.def("readTensor", [](PySharsorIPC::Wrapper& wrapper,
                             pybind11::array& np_array,
                             int row, int col) {

        return wrapper.execute([&](pybind11::object& client) -> bool {

            // we need to check data compatibility at runtime
            // (this was not necessary in C++, since it's something
            // checked at compile time)
            // this introduces a slight overhead, but avoids unpredicted
            // behavior when the user passes non-compatible types

            // and of the input np array
            pybind11::dtype np_dtype = np_array.dtype();

            switch (client.attr("getScalarType")().cast<DType>()) {

            case DType::Bool:

                if (!np_dtype.is(pybind11::dtype::of<bool>())) {

                    std::string error = std::string("Mismatched dtype: expected bool numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);

                }

                break;

            case DType::Int:

                if (!np_dtype.is(pybind11::dtype::of<int>())) {

                    std::string error = std::string("Mismatched dtype: expected int numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            case DType::Float:

                if (!np_dtype.is(pybind11::dtype::of<float>())) {

                    std::string error = std::string("Mismatched dtype: expected float numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            case DType::Double:

                if (!np_dtype.is(pybind11::dtype::of<double>())) {

                    std::string error = std::string("Mismatched dtype: expected double numpy array but got ") +
                                    pybind11::str(np_dtype).cast<std::string>();

                    SharsorIPCpp::Journal::log("Client",
                                 "readTensor",
                                 error,
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

    }, pybind11::arg("tensor"), pybind11::arg("row") = 0, pybind11::arg("col") = 0);
}

void PySharsorIPC::bindClients(pybind11::module& m) {

    bindClientT<bool, SharsorIPCpp::ColMajor>(m, "PyClientBoolColMaj");
    bindClientT<bool, SharsorIPCpp::RowMajor>(m, "PyClientBoolRowMaj");

    bindClientT<int, SharsorIPCpp::ColMajor>(m, "PyClientIntColMaj");
    bindClientT<int, SharsorIPCpp::RowMajor>(m, "PyClientIntRowMaj");

    bindClientT<float, SharsorIPCpp::ColMajor>(m, "PyClientFloatColMaj");
    bindClientT<float, SharsorIPCpp::RowMajor>(m, "PyClientFloatRowMaj");

    bindClientT<double, SharsorIPCpp::ColMajor>(m, "PyClientDoubleColMaj");
    bindClientT<double, SharsorIPCpp::RowMajor>(m, "PyClientDoubleRowMaj");

}

void PySharsorIPC::bindClientFactory(pybind11::module& m,
                           const char* name) {

    m.def(name, &ClientFactory,
          pybind11::arg("basename") = "MySharedMemory",
          pybind11::arg("namespace") = "",
          pybind11::arg("verbose") = false,
          pybind11::arg("vlevel") = VLevel::V0,
          pybind11::arg("dtype") = DType::Float,
          pybind11::arg("layout") = SharsorIPCpp::RowMajor, // default of numpy and pytorch
          "Create a new client with the specified arguments and dtype.");

}
