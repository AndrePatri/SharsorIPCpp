#include <PySharsorIPC/PyServer.hpp>

template <typename Scalar>
void bind_ServerT(py::module &m, const char* name) {
    py::class_<Server<Scalar>, std::shared_ptr<Server<Scalar>>>(m, name)
        .def(py::init<int, int, std::string, std::string, bool, VLevel, bool>())

        // Wrapped writeTensor method for Python
        .def("writeTensor", [](Server<Scalar>& server,
                               const py::array_t<Scalar>& np_array,
                               int row, int col) {

            // Convert numpy array to Eigen matrix
            Tensor<Scalar> tensor = py::cast<Tensor<Scalar>>(np_array);

            // Call the original writeTensor method
            return server.writeTensor(tensor, row, col);

        }, py::arg("data"), py::arg("row") = 0, py::arg("col") = 0)

        // Wrapped readTensor method for Python
        .def("readTensor", [](Server<Scalar>& server, int row, int col) {

            Tensor<Scalar> output;

            bool success = server.readTensor(output, row, col);

            return py::make_tuple(success, output);

        }, py::arg("row") = 0, py::arg("col") = 0)

        .def("run", &Server<Scalar>::run)

        .def("stop", &Server<Scalar>::stop)

        .def("close", &Server<Scalar>::close)

        .def("isRunning", &Server<Scalar>::isRunning)

        .def("getNClients", &Server<Scalar>::getNClients)

        .def("getNRows", &Server<Scalar>::getNRows)

        .def("getNCols", &Server<Scalar>::getNCols);
}

void bind_Server(py::module &m) {
    py::enum_<SrvrType>(m, "SrvrType")
        .value("BOOL", SrvrType::BOOL)
        .value("INT", SrvrType::INT)
        .value("FLOAT", SrvrType::FLOAT)
        .value("DOUBLE", SrvrType::DOUBLE);

    bind_ServerT<bool>(m, "ServerBool");
    bind_ServerT<int>(m, "ServerInt");
    bind_ServerT<float>(m, "ServerFloat");
    bind_ServerT<double>(m, "ServerDouble");
}
