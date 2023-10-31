#include <PySharsorIPC/PyClient.hpp>
#include <SharsorIPCpp/Journal.hpp>

template <typename Scalar, int Layout>
void PySharsorIPC::bindClientT(py::module &m, const char* name) {

    // bindings of Client for Python

    using Journal = SharsorIPCpp::Journal;

    py::class_<SharsorIPCpp::Client<Scalar, Layout>,
            std::shared_ptr<SharsorIPCpp::Client<Scalar, Layout>>>(m, name)

        .def(py::init<std::string, std::string, bool, SharsorIPCpp::VLevel>())

//        .def("writeTensor", [](SharsorIPCpp::Client<Scalar, Layout>& self,
//                       PySharsorIPC::NumpyArray<Scalar, Layout>& arr,
//                       int row, int col) {

//            // Check dtype
//            if (!arr.dtype().is(py::dtype::of<Scalar>())) {

//                std::string expected_dtype = py::format_descriptor<Scalar>::format();
//                std::string given_dtype(1, arr.dtype().kind());
//                std::string errorMsg = "Data type mismatch. Expected dtype: " +
//                        expected_dtype +
//                        ", but got: " +
//                        given_dtype + ".";

//                SharsorIPCpp::Journal::log("PyClient", "writeTensor",
//                                           errorMsg,
//                                           SharsorIPCpp::Journal::LogType::EXCEP,
//                                           true // throw exception
//                                           );

//                throw std::runtime_error("Invalid data type.");

//            }

//            // Check layout compatibility
//            if (Layout == SharsorIPCpp::ColMajor && !(arr.flags() & py::array::f_style)) {

//                Journal::log("PyClient",
//                             "writeTensor",
//                             "Expected a column-major (Fortran-contiguous) array.",
//                             Journal::LogType::EXCEP,
//                             true // throw exception
//                             );

//            }
//            else if (Layout == SharsorIPCpp::RowMajor && !(arr.flags() & py::array::c_style)) {


//                Journal::log("PyClient",
//                             "writeTensor",
//                             "Expected a row-major (C-contiguous)  array.",
//                             Journal::LogType::EXCEP,
//                             true // throw exception
//                             );

//            }

//            // will not create a dynamic allocation
//            Eigen::Ref<SharsorIPCpp::Tensor<Scalar, Layout>> tensor_ref = arr;

//            return self.writeTensor(tensor_ref, row, col);

//        })

        .def("readTensor", [](SharsorIPCpp::Client<Scalar, Layout>& self,
                       PySharsorIPC::NumpyArray<Scalar>& arr,
                       int row, int col) {

            // Check dtype
            if (!arr.dtype().is(py::dtype::of<Scalar>())) {

                std::string expected_dtype = py::format_descriptor<Scalar>::format();
                std::string given_dtype(1, arr.dtype().kind());
                std::string errorMsg = "Data type mismatch. Expected dtype: " +
                        expected_dtype +
                        ", but got: " +
                        given_dtype + ".";

                SharsorIPCpp::Journal::log("PyClient", "readTensor",
                                           errorMsg,
                                           SharsorIPCpp::Journal::LogType::EXCEP,
                                           true // throw exception
                                           );

                throw std::runtime_error("Invalid data type.");

            }

            auto np_strides = arr.strides();

            int stride_row = np_strides[0] / sizeof(Scalar);
            int stride_col = np_strides[1] / sizeof(Scalar);

            std::cout << "strides: " << stride_row << stride_col << std::endl;
//            std::cout << "Is view "<< !arr.attr("base").is_none() << std::endl;
//            std::cout << "Stride first  "<< np_strides.first << "Stride second" << np_strides.second << std::endl;

//            if (!(arr.flags() & py::array::f_style) && // nor f or c style (it's probably a view)
//                !(arr.flags() & py::array::c_style)) {

//                auto strides = arr.attr("strides").cast<std::pair<int, int>>();

//            }

            // Check layout compatibility
            if (Layout == SharsorIPCpp::ColMajor && !(arr.flags() & py::array::f_style)) {

                Journal::log("PyClient",
                             "readTensor",
                             "Expected a column-major (Fortran-contiguous) array",
                             Journal::LogType::EXCEP,
                             true // throw exception
                             );

            }
            else if (Layout == SharsorIPCpp::RowMajor && !(arr.flags() & py::array::c_style)) {


                Journal::log("PyClient",
                             "readTensor",
                             "Expected a row-major (C-contiguous)  array",
                             Journal::LogType::EXCEP,
                             true // throw exception
                             );

            }

            // Ensure the numpy array is mutable and get a pointer to its data
            py::buffer_info buf_info = arr.request();

            bool success = false;

            Scalar* start_ptr = static_cast<Scalar*>(buf_info.ptr);

            SharsorIPCpp::DStrides strides;

//            SharsorIPCpp::DStrides strides(np_strides.first,
//                                           np_strides.second);

            if (Layout == SharsorIPCpp::ColMajor){

                strides = SharsorIPCpp::DStrides(arr.shape(0),
                                                 1);
            }
            if (Layout == SharsorIPCpp::RowMajor){

                strides = SharsorIPCpp::DStrides(arr.shape(1),
                                                 1);
            }

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

void PySharsorIPC::bindClients(py::module& m) {

    bindClientT<bool, SharsorIPCpp::ColMajor>(m, "__PyClientBoolColMaj");
    bindClientT<bool, SharsorIPCpp::RowMajor>(m, "__PyClientBoolRowMaj");

    bindClientT<int, SharsorIPCpp::ColMajor>(m, "__PyClientIntColMaj");
    bindClientT<int, SharsorIPCpp::RowMajor>(m, "__PyClientIntRowMaj");

    bindClientT<float, SharsorIPCpp::ColMajor>(m, "__PyClientFloatColMaj");
    bindClientT<float, SharsorIPCpp::RowMajor>(m, "__PyClientFloatRowMaj");

    bindClientT<double, SharsorIPCpp::ColMajor>(m, "__PyClientDoubleColMaj");
    bindClientT<double, SharsorIPCpp::RowMajor>(m, "__PyClientDoubleRowMaj");

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
