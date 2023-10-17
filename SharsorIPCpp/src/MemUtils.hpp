#ifndef MEMUTILS_HPP
#define MEMUTILS_HPP

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <csignal>
#include <memory>

#include <SharsorIPCpp/Journal.hpp>

namespace SharsorIPCpp{

    template <typename Scalar>
    using Tensor = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    template <typename Scalar>
    using MMap = Eigen::Map<Tensor<Scalar>>;

    using VLevel = Journal::VLevel;

    using LogType = Journal::LogType;

    namespace MemUtils{

        template <typename Scalar>
        void initMem(
            std::size_t n_rows,
            std::size_t n_cols,
            const std::string& mem_path,
            int& shm_fd,
            MMap<Scalar>& tensor_view,
            Journal& journal,
            bool verbose = true,
            VLevel vlevel = Journal::VLevel::V0)
        {

            // Determine the size based on the Scalar type
            std::size_t data_size = sizeof(Scalar) * n_rows * n_cols;

            // Create shared memory
            shm_fd = shm_open(mem_path.c_str(),
                              O_CREAT | O_RDWR,
                              S_IRUSR | S_IWUSR);

            if (shm_fd == -1) {

                std::string error = "Could not create shared memory at " +
                        mem_path;

                journal.log(__FUNCTION__,
                    error,
                    LogType::EXCEP);

            }

            // Set size
            if (ftruncate(shm_fd, data_size) == -1) {

                std::string error = "Could not set shared memory at " +
                        mem_path;

                journal.log(__FUNCTION__,
                            error,
                            LogType::EXCEP);

            }

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = "Opened shared memory at " +
                        mem_path;

                journal.log(__FUNCTION__,
                            info,
                            LogType::INFO);
            }

            // Map the shared memory
            Scalar* matrix_data = static_cast<Scalar*>(mmap(nullptr,
                                                        data_size,
                                                        PROT_READ | PROT_WRITE,
                                                        MAP_SHARED,
                                                        shm_fd,
                                                        0));

            if (matrix_data == MAP_FAILED) {

                journal.log(__FUNCTION__,
                            "Could not map memory size.",
                            LogType::EXCEP);

            }

            new (&tensor_view) MMap<Scalar>(matrix_data,
                                           n_rows,
                                           n_cols);

            if (verbose && vlevel > VLevel::V2) {

                std::string info = "Mapped shared memory at " +
                        mem_path;

                journal.log(__FUNCTION__,
                            info,
                            LogType::INFO);

            }

        }

    }

}

#endif // MEMUTILS_HPP
