#ifndef DTYPES_HPP
#define DTYPES_HPP

namespace SharsorIPCpp {

    template <typename Scalar>
    using Tensor = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic,
                                    Eigen::RowMajor>; // to ensure compatibility
    // with col. major libraries like Numpy or PyTorch

    template <typename Scalar>
    using MMap = Eigen::Map<Tensor<Scalar>>; // no explicit cleanup needed
    // for Eigen::Map -> it does not own the memory

    // Define an enum class for data types
    enum class DType {
        Float,
        Double,
        Int,
        Bool
    };

    // Create a type trait to map DType values to C++ types
    template <DType>
    struct DTypeToCppType;

    // Specialize the trait for each DType
    template <>
    struct DTypeToCppType<DType::Float> {
        using type = float;
    };

    template <>
    struct DTypeToCppType<DType::Double> {
        using type = double;
    };

    template <>
    struct DTypeToCppType<DType::Int> {
        using type = int;
    };

    template <>
    struct DTypeToCppType<DType::Bool> {
        using type = bool;
    };

    // Create a type trait to map C++ types values to  DTypes
    template <typename T>
    struct CppTypeToDType;

    template <>
    struct CppTypeToDType<float> {
        static constexpr DType value = DType::Float;
    };

    template <>
    struct CppTypeToDType<double> {
        static constexpr DType value = DType::Double;
    };

    template <>
    struct CppTypeToDType<int> {
        static constexpr DType value = DType::Int;
    };

    template <>
    struct CppTypeToDType<bool> {
        static constexpr DType value = DType::Bool;
    };

}

#endif // DTYPES_HPP

