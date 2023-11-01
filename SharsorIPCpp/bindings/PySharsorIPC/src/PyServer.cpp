#include <PySharsorIPC/PyServer.hpp>

template <typename Scalar, int Layout>
void PySharsorIPC::bindServerT(pybind11::module &m, const char* name) {

    // bindings of Server for Python

    pybind11::class_<SharsorIPCpp::Server<Scalar, Layout>,
            std::shared_ptr<SharsorIPCpp::Server<Scalar, Layout>>>(m, name)

        .def(pybind11::init<int, int, std::string, std::string, bool, SharsorIPCpp::VLevel>())

        .def("writeTensor", [](SharsorIPCpp::Server<Scalar, Layout>& self,
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

        .def("readTensor", [](SharsorIPCpp::Server<Scalar, Layout>& self,
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

        .def("run", &SharsorIPCpp::Server<Scalar, Layout>::run)

        .def("stop", &SharsorIPCpp::Server<Scalar, Layout>::stop)

        .def("close", &SharsorIPCpp::Server<Scalar, Layout>::close)

        .def("isRunning", &SharsorIPCpp::Server<Scalar, Layout>::isRunning)

        .def("getScalarType", &SharsorIPCpp::Server<Scalar, Layout>::getScalarType)

        .def("getNRows", &SharsorIPCpp::Server<Scalar, Layout>::getNRows)

        .def("getNCols", &SharsorIPCpp::Server<Scalar, Layout>::getNCols);
}

pybind11::object PySharsorIPC::ServerFactory(int n_rows,
                                       int n_cols,
                                    std::string basename,
                                    std::string name_space,
                                    bool verbose,
                                    VLevel vlevel,
                                    DType dtype,
                                    int layout) {

    switch (layout) {

        case SharsorIPCpp::ColMajor:

            switch (dtype) {

                case DType::Bool:

                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                          pybind11::cast(std::make_shared<SharsorIPCpp::Server<bool, SharsorIPCpp::ColMajor>>(n_rows,
                                                                                                                        n_cols,
                                                                                                                        basename,
                                                                                                                        name_space,
                                                                                                                        verbose,
                                                                                                                        vlevel)))));
                case DType::Int:

                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Server<int, SharsorIPCpp::ColMajor>>(n_rows,
                                                                                                                                   n_cols,
                                                                                                                                   basename,
                                                                                                                                   name_space,
                                                                                                                                   verbose,
                                                                                                                                   vlevel)))));
                case DType::Float:

                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Server<float, SharsorIPCpp::ColMajor>>(n_rows,
                                                                                                                                     n_cols,
                                                                                                                                     basename,
                                                                                                                                     name_space,
                                                                                                                                     verbose,
                                                                                                                                     vlevel)))));
                case DType::Double:

                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Server<double, SharsorIPCpp::ColMajor>>(n_rows,
                                                                                                                                      n_cols,
                                                                                                                                      basename,
                                                                                                                                      name_space,
                                                                                                                                      verbose,
                                                                                                                                      vlevel)))));
                default:

                    throw std::runtime_error("Invalid dtype specified!");
            }

        case SharsorIPCpp::RowMajor:

            switch (dtype) {

                case DType::Bool:

                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Server<bool, SharsorIPCpp::RowMajor>>(n_rows,
                                                                                                                                    n_cols,
                                                                                                                                    basename,
                                                                                                                                    name_space,
                                                                                                                                    verbose,
                                                                                                                                    vlevel)))));
                case DType::Int:
                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Server<int, SharsorIPCpp::RowMajor>>(n_rows,
                                                                                                                                   n_cols,
                                                                                                                                   basename,
                                                                                                                                   name_space,
                                                                                                                                   verbose,
                                                                                                                                   vlevel)))));
                case DType::Float:
                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Server<float, SharsorIPCpp::RowMajor>>(n_rows,
                                                                                                                                     n_cols,
                                                                                                                                     basename,
                                                                                                                                     name_space,
                                                                                                                                     verbose,
                                                                                                                                     vlevel)))));
                case DType::Double:
                    return pybind11::cast(PySharsorIPC::ServerWrapper(new pybind11::object(
                                                      pybind11::cast(std::make_shared<SharsorIPCpp::Server<double, SharsorIPCpp::RowMajor>>(n_rows,
                                                                                                                                      n_cols,
                                                                                                                                      basename,
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

void PySharsorIPC::bind_ServerWrapper(pybind11::module& m) {

    // By handling everything in Python-space,
    // we eliminate the need to handle the type
    // conversion in C++, making the ServerWrapper
    // agnostic to the actual data type of the underlying Server.

    // Here we lose a bit of performance wrt using the bare class bindings,
    // since additional logic and checks are necessary. However, we gain
    // much cleaner interface and flexibility

    // (here we are calling python code from C++)

    pybind11::class_<PySharsorIPC::ServerWrapper> cls(m, "Server");

    cls.def("run", [](PySharsorIPC::ServerWrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& server) {

            return server.attr("run")();

        });

    });

    cls.def("stop", [](PySharsorIPC::ServerWrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& server) {

            return server.attr("stop")();

        });

    });

    cls.def("close", [](PySharsorIPC::ServerWrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& server) {

            return server.attr("close")();

        });

    });

    cls.def("isRunning", [](PySharsorIPC::ServerWrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& server) {

            return server.attr("isAttached")().cast<bool>();

        });

    });

    cls.def("getNRows", [](PySharsorIPC::ServerWrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& server) {

            return server.attr("getNRows")().cast<int>();

        });

    });

    cls.def("getNCols", [](PySharsorIPC::ServerWrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& server) {

            return server.attr("getNCols")().cast<int>();

        });

    });

    cls.def("getScalarType", [](PySharsorIPC::ServerWrapper& wrapper) {

        return wrapper.execute([&](pybind11::object& server) {

            return server.attr("getScalarType")().cast<DType>();

        });

    });

    cls.def("writeTensor", [](PySharsorIPC::ServerWrapper& wrapper,
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

    cls.def("readTensor", [](PySharsorIPC::ServerWrapper& wrapper,
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

                    SharsorIPCpp::Journal::log("Server",
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

                    SharsorIPCpp::Journal::log("Server",
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

                    SharsorIPCpp::Journal::log("Server",
                                 "readTensor",
                                 error,
                                 SharsorIPCpp::LogType::EXCEP,
                                 true);
                }

                break;

            default:

                SharsorIPCpp::Journal::log("Server",
                             "readTensor",
                             "Mismatched dtype: provided dtype not supported.",
                             SharsorIPCpp::LogType::EXCEP,
                             true);

            }

            // now we can safely read the tensor
            return server.attr("readTensor")(np_array,
                                             row, col).cast<bool>();

        });

    }, pybind11::arg("tensor"), pybind11::arg("row") = 0, pybind11::arg("col") = 0);
}

void PySharsorIPC::bindServers(pybind11::module& m) {

    bindServerT<bool, SharsorIPCpp::ColMajor>(m, "PyServerBoolColMaj");
    bindServerT<bool, SharsorIPCpp::RowMajor>(m, "PyServerBoolRowMaj");

    bindServerT<int, SharsorIPCpp::ColMajor>(m, "PyServerIntColMaj");
    bindServerT<int, SharsorIPCpp::RowMajor>(m, "PyServerIntRowMaj");

    bindServerT<float, SharsorIPCpp::ColMajor>(m, "PyServerFloatColMaj");
    bindServerT<float, SharsorIPCpp::RowMajor>(m, "PyServerFloatRowMaj");

    bindServerT<double, SharsorIPCpp::ColMajor>(m, "PyServerDoubleColMaj");
    bindServerT<double, SharsorIPCpp::RowMajor>(m, "PyServerDoubleRowMaj");

}

void PySharsorIPC::bindServerFactory(pybind11::module& m,
                           const char* name) {

    m.def(name, &ServerFactory,
          pybind11::arg("n_rows"),
          pybind11::arg("n_cols"),
          pybind11::arg("basename"),
          pybind11::arg("namespace") = "",
          pybind11::arg("verbose") = false,
          pybind11::arg("vlevel") = VLevel::V0,
          pybind11::arg("dtype") = DType::Float,
          pybind11::arg("layout") = SharsorIPCpp::RowMajor, // default of numpy and pytorch
          "Create a new server with the specified arguments and dtype.",
          pybind11::return_value_policy::reference_internal); // reference_internal keeps the underlying object alive,
          // as long as the python is


}
