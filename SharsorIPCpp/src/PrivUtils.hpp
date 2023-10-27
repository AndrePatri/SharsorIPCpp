#ifndef PRIVUTILS_H
#define PRIVUTILS_H

#include <SharsorIPCpp/DTypes.hpp>

namespace SharsorIPCpp {

    using Index = Eigen::Index;

    using VLevel = Journal::VLevel;

    using LogType = Journal::LogType;

    inline bool canFitTensor(Index n_rows,
                    Index n_cols,
                    int i, int j,
                    Index n_rows2fit,
                    Index n_cols2fit,
                    Journal& journal,
                    ReturnCode& return_code,
                    bool verbose = true,
                    VLevel vlevel = Journal::VLevel::V0) {

        // Check if the indices (i, j) are within the matrix
        if (i < 0 || i >= n_rows || j < 0 || j >= n_cols) {

            if (verbose &&
                    vlevel > VLevel::V0) {

                std::string warn =
                        std::string("Provided indeces are out of bounds wrt memory.");

                journal.log(__FUNCTION__,
                            warn,
                            LogType::EXCEP);

            }

            return_code = return_code + ReturnCode::INDXOUT;

            return false;
        }

        // Check if there's enough space for the submatrix
        if (i + n_rows2fit > n_rows ||
            j + n_cols2fit > n_cols) {

            if (verbose &&
                    vlevel > VLevel::V0) {

                std::string warn =
                        std::string("Out of bounds dimensions!");

                journal.log(__FUNCTION__,
                             warn,
                             LogType::EXCEP);

            }

            return_code = return_code + ReturnCode::NOFIT;

            return false;
        }

        return true;
    }

}

#endif // PRIVUTILS_H
