#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <Eigen/Dense>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

namespace SharsorIPCpp {

    namespace helpers{

        template <typename Scalar>
        void createViewFrom(MMap<Scalar>& view,
                            const Tensor<Scalar>& from,
                            int row_idx, int col_idx,
                            int n_rows, int n_cols);

        template <typename Scalar>
        MMap<Scalar> createViewFrom(const Tensor<Scalar>& from,
                                    int row_idx, int col_idx,
                                    int n_rows, int n_cols);
    }

}

#endif // HELPERS_HPP
