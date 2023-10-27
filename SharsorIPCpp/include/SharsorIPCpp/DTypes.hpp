#ifndef DTYPES_HPP
#define DTYPES_HPP

namespace SharsorIPCpp {

    const int RowMajor = Eigen::RowMajor;
    const int ColMajor = Eigen::ColMajor;

    using DStrides = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;

    const int MemLayoutDefault = RowMajor; // default layout used by this
    // library (changes here will propagate to the whole library)

    template <typename Scalar, int Layout = MemLayoutDefault>
    using Tensor = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Layout>;

    template <typename Scalar, int Layout = MemLayoutDefault>
    using DMMap = Eigen::Map<Tensor<Scalar, Layout>, // matrix type
                        Eigen::Unaligned, // specifies the pointer alignment in bytes
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>; // specifies strides
    // By default, Map assumes that the data is laid out contiguously in memory

    template <typename Scalar, int Layout = MemLayoutDefault>
    using MMap = Eigen::Map<Tensor<Scalar, Layout>>; // no explicit cleanup needed
    // for Eigen::Map -> it does not own the memory.

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

