#include "Server.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <cstring> // for std::strerror()
#include <typeinfo>

namespace SharsorIPCpp {

    // Define a type trait to check if a given type is a valid DType
    template <typename Scalar>
    struct IsValidDType {
        static constexpr bool value =
            std::is_same<Scalar, typename DTypeToCppType<DType::Float>::type>::value ||
            std::is_same<Scalar, typename DTypeToCppType<DType::Double>::type>::value ||
            std::is_same<Scalar, typename DTypeToCppType<DType::Int>::type>::value ||
            std::is_same<Scalar, typename DTypeToCppType<DType::Bool>::type>::value;
    };

    template <typename Scalar>
    Server<Scalar>::Server(int n_rows,
                   int n_cols,
                   std::string memname,
                   std::string name_space)
        : n_rows(n_rows),
          n_cols(n_cols),
          _shared_mem_name(memname),
          _namespace{name_space},
          _tensor_view(nullptr,
                       n_rows,
                       n_rows),
          _journal(Journal(GetThisName()))
    {

        static_assert(IsValidDType<Scalar>::value, "Invalid data type provided.");

        // Determine the size based on the Scalar type
        std::size_t data_size = sizeof(Scalar) * n_rows * n_cols;

        // Create shared memory
        _shm_fd = shm_open(_shared_mem_name.c_str(),
                           O_CREAT | O_RDWR,
                           S_IRUSR | S_IWUSR);

        if (_shm_fd == -1) {

            throw std::runtime_error("Cannot create shared memory: " +
                                     std::string(std::strerror(errno)));
        }

        // Set size
        if (ftruncate(_shm_fd, data_size) == -1) {

            throw std::runtime_error("Cannot set shared memory size: " +
                                     std::string(std::strerror(errno)));

        }

        // Map the shared memory
        Scalar* matrix_data = static_cast<Scalar*>(mmap(nullptr, data_size,
                                                        PROT_READ | PROT_WRITE,
                                                        MAP_SHARED, _shm_fd, 0));
        if (matrix_data == MAP_FAILED) {

            throw std::runtime_error("Cannot map shared memory: " +
                                     std::string(std::strerror(errno)));
        }

        new (&_tensor_view) MMap<Scalar>(matrix_data, n_rows, n_cols);

        _tensor_copy = Tensor<Scalar>::Zero(n_rows, n_cols);
    }

    template <typename Scalar>
    Server<Scalar>::~Server() {

        shm_unlink(_shared_mem_name.c_str());

    }

    template <typename Scalar>
    std::string Server<Scalar>::GetThisName()
    {
        const std::type_info& info = typeid(*this);

        return std::string(info.name());
    }

    template <typename Scalar>
    void Server<Scalar>::writeMemory(const Tensor<Scalar>& data) {

        if(data.rows() != n_rows || data.cols() != n_cols) {

            throw std::runtime_error("Data dimensions mismatch");

        }

        _tensor_view.block(0, 0, n_rows, n_cols) = data;
    }

    template <typename Scalar>
    const MMap<Scalar>& Server<Scalar>::getTensorView() {

        return _tensor_view;

    }

    template <typename Scalar>
    const Tensor<Scalar> &Server<Scalar>::getTensorCopy() {

        _tensor_copy = _tensor_view;

        return _tensor_copy;

    }

    // explicit instantiations for specific types
    template class Server<double>;
    template class Server<float>;
    template class Server<int>;
    template class Server<bool>;
}
