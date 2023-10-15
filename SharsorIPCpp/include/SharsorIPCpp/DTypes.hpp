#ifndef DTYPES_HPP
#define DTYPES_HPP

namespace SharsorIPCpp {

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

}

#endif // DTYPES_HPP

