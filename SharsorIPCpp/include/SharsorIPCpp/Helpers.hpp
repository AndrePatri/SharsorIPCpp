#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <Eigen/Dense>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

namespace SharsorIPCpp {

    namespace helpers{

//        template <typename Scalar>
//        void createViewFrom(MMap<Scalar>& view,
//                            Tensor<Scalar>& from,
//                            int row_idx, int col_idx,
//                            int n_rows, int n_cols);

        template <typename Scalar>
        MMap<Scalar> createViewFrom(Tensor<Scalar>& from,
                                    int row_idx, int col_idx,
                                    int n_rows, int n_cols);


//        template <typename Scalar>
//        void helpers::createViewFrom(MMap<Scalar>& view,
//                            Tensor<Scalar>& from,
//                            int row_idx, int col_idx,
//                            int n_rows, int n_cols) {

//            // Calculating the pointer to the starting position of the block in memory
//            Scalar* startPtr = from.data() +
//                               row_idx * from.outerStride() +
//                               col_idx * from.innerStride();

//            // Setup the map view with appropriate sizes and strides
//            view = MMap<Scalar>(startPtr,
//                                n_rows, n_cols,
//                                Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(from.outerStride(),
//                                                                              from.innerStride())
//                                );
//        }

        template <typename Scalar>
        MMap<Scalar> helpers::createViewFrom(
                                    Tensor<Scalar>& from,
                                    int row_idx, int col_idx,
                                    int n_rows, int n_cols) {

            // Calculating the pointer to the starting position of the block in memory

            Scalar* startPtr;

            if (Tensor<Scalar>::IsRowMajor) {

//              check if a matrix (or tensor) is row-major or column-major at compile time

                startPtr = from.data() + // pointer to first element of from
                           row_idx * from.outerStride() +
                           col_idx * from.innerStride();

            } else {

                startPtr = from.data() + // pointer to first element of from
                           col_idx * from.outerStride() +
                           row_idx * from.innerStride();

            }

            // Return the map view with appropriate sizes and strides

            // Create the view using Eigen::Map
            return MMap<Scalar>(startPtr, n_rows, n_cols); // this will always match
            // the right  ordering at compile time, since its defined based on Tensor

        }
    }

}

#endif // HELPERS_HPP
