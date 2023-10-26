#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <Eigen/Dense>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

namespace SharsorIPCpp {

    namespace helpers{

        template <typename Scalar, int Layout = MemLayoutDefault>
        DMMap<Scalar, Layout> createViewFrom(
                                    Tensor<Scalar, Layout>& from,
                                    int row_idx, int col_idx,
                                    int n_rows, int n_cols);

        template <typename Scalar, int Layout>
        DMMap<Scalar, Layout> createViewFrom(
            Tensor<Scalar, Layout>& from,
            int row_idx, int col_idx,
            int n_rows, int n_cols) {

            // Manually compute the starting pointer for the block, taking the layout into account

            Scalar* startPtr;

            if (Layout == SharsorIPCpp::ColMajor) {

                startPtr = from.data() +
                        col_idx * from.rows() + row_idx;

                // Define the stride based on the layout of the matrix
                DStrides stride(from.rows(),
                                1);

                // Use the mapped view with the provided stride to interpret the block correctly
                return DMMap<Scalar, Layout>(startPtr,
                             n_rows, n_cols,
                             stride);

            } else {

                startPtr = from.data() +
                        row_idx * from.cols() + col_idx;

                // Define the stride based on the layout of the matrix
                DStrides stride(1,
                                from.cols());

                // Use the mapped view with the provided stride to interpret the block correctly
                return DMMap<Scalar, Layout>(startPtr,
                             n_rows, n_cols,
                             stride);
            }

        }

    }

}

#endif // HELPERS_HPP
