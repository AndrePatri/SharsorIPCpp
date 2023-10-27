#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <Eigen/Dense>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>
#include <PrivUtils.hpp>
#include <SharsorIPCpp/ReturnCodes.hpp>

namespace SharsorIPCpp {

    namespace helpers{

        template <typename Scalar, int Layout = MemLayoutDefault>
        TensorView<Scalar, Layout> createViewFrom(
                                    Tensor<Scalar, Layout>& from,
                                    int row_idx, int col_idx,
                                    int n_rows, int n_cols);

        template <typename Scalar, int Layout>
        TensorView<Scalar, Layout> createViewFrom(
            Tensor<Scalar, Layout>& from,
            int row_idx, int col_idx,
            int n_rows, int n_cols) {

            // note: this is not meant to be used in RT and hence should only be called during
            // initialization phases.

            ReturnCode return_code = ReturnCode::RESET; // unused; just to call the method

            Journal journal = Journal("");

            bool success = canFitTensor( // we check that are creating a valid view of "from"
                         from.rows(),
                         from.cols(),
                         row_idx, col_idx,
                         n_rows, n_cols,
                         journal,
                         return_code,
                         true,
                         VLevel::V3);

            if (!success) {

                std::string excep =
                        std::string("View does not fit in original tensor!!!");

                journal.log(__FUNCTION__,
                             excep,
                             LogType::EXCEP,
                             true // throw runtime exception
                             );
            } // if ok, we go on

            Scalar* startPtr; // start pointer from which to create the view

            if (Layout == SharsorIPCpp::ColMajor) {

                startPtr = from.data() +
                        col_idx * from.rows() + row_idx;

                // Define the stride based on the layout of the matrix
                DStrides stride(from.rows(),
                                1);

                // Use the mapped view with the provided stride to interpret the block correctly
                return TensorView<Scalar, Layout>(startPtr,
                             n_rows, n_cols,
                             stride);

            } else {

                std::cout << "#############à" << std::endl;
                startPtr = from.data() +
                        row_idx * from.cols() + col_idx;

                // Define the stride based on the layout of the matrix
                DStrides stride(from.cols(),
                                1);

                // Use the mapped view with the provided stride to interpret the block correctly
                return TensorView<Scalar, Layout>(startPtr,
                             n_rows, n_cols,
                             stride);
            }

        }

    }

}

#endif // HELPERS_HPP
