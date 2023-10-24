#include <SharsorIPCpp/Helpers.hpp>

namespace SharsorIPCpp {

    template <typename Scalar>
    void helpers::createViewFrom(MMap<Scalar>& view,
                        Tensor<Scalar>& from,
                        int row_idx, int col_idx,
                        int n_rows, int n_cols) {

        // Calculating the pointer to the starting position of the block in memory
        Scalar* startPtr = from.data() +
                           row_idx * from.outerStride() +
                           col_idx * from.innerStride();

        // Setup the map view with appropriate sizes and strides
        view = MMap<Scalar>(startPtr,
                            n_rows, n_cols,
                            Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(from.outerStride(),
                                                                          from.innerStride())
                            );
    }

    template <typename Scalar>
    SharsorIPCpp::MMap<Scalar> helpers::createViewFrom(
                                SharsorIPCpp::Tensor<Scalar>& from,
                                int row_idx, int col_idx,
                                int n_rows, int n_cols) {

        // Calculating the pointer to the starting position of the block in memory

        Scalar* startPtr = from.data() +
                row_idx * from.outerStride() +
                col_idx * from.innerStride();

        // Return the map view with appropriate sizes and strides

        // Create the view using Eigen::Map
        return SharsorIPCpp::MMap<Scalar>(startPtr, n_rows, n_cols);

    }

    // explicit instantiations for specific supported types
//    template MMap<bool> helpers::createViewFrom(const Tensor<bool>& from,
//                                         int row_idx, int col_idx,
//                                         int n_rows, int n_cols);
//    template MMap<int> helpers::createViewFrom(Tensor<int>& from,
//                                         int row_idx, int col_idx,
//                                         int n_rows, int n_cols);
    template SharsorIPCpp::MMap<float> helpers::createViewFrom(const SharsorIPCpp::Tensor<float>& from,
                                         int row_idx, int col_idx,
                                         int n_rows, int n_cols);
//    template MMap<double> helpers::createViewFrom(const Tensor<double>& from,
//                                         int row_idx, int col_idx,
//                                         int n_rows, int n_cols);

}




