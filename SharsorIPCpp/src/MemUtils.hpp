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

#include <SharsorIPCpp/DTypes.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/ReturnCodes.hpp>

namespace SharsorIPCpp{

    template <typename Scalar>
    using Tensor = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    template <typename Scalar>
    using MMap = Eigen::Map<Tensor<Scalar>>; // no explicit cleanup needed
    // for Eigen::Map -> it does not own the memory

    using VLevel = Journal::VLevel;

    using LogType = Journal::LogType;

    using Index = Eigen::Index;

    namespace MemUtils{

        // Define a type trait to check if a given type is a valid DType
        template <typename Scalar>
        struct IsValidDType {
            static constexpr bool value =
                std::is_same<Scalar, typename DTypeToCppType<DType::Float>::type>::value ||
                std::is_same<Scalar, typename DTypeToCppType<DType::Double>::type>::value ||
                std::is_same<Scalar, typename DTypeToCppType<DType::Int>::type>::value ||
                std::is_same<Scalar, typename DTypeToCppType<DType::Bool>::type>::value;
        };

        template <typename Scalar>
        void initMem(
            std::size_t n_rows,
            std::size_t n_cols,
            const std::string& mem_path,
            int& shm_fd,
            MMap<Scalar>& tensor_view,
            Journal& journal,
            ReturnCode& return_code,
            bool verbose = true,
            VLevel vlevel = Journal::VLevel::V0
            )
        {
            return_code = ReturnCode::RESET; // resets return code

            // Determine the size based on the Scalar type
            std::size_t data_size = sizeof(Scalar) * n_rows * n_cols;

            // Create shared memory
            shm_fd = shm_open(mem_path.c_str(),
                              O_CREAT | O_RDWR,
                              S_IRUSR | S_IWUSR);

            if (shm_fd == -1) {

                if (verbose) {

                    std::string error = "Could not create shared memory at " +
                            mem_path;

                    journal.log(__FUNCTION__,
                        error,
                        LogType::EXCEP);
                }

                return_code = return_code + ReturnCode::MEMCREATFAIL;

            }

            // Set size
            if (ftruncate(shm_fd, data_size) == -1) {

                if (verbose) {

                    std::string error = "Could not set shared memory at " +
                            mem_path;

                    journal.log(__FUNCTION__,
                                error,
                                LogType::EXCEP);
                }

                return_code = return_code + ReturnCode::MEMSETFAIL;

            }

            return_code = return_code + ReturnCode::MEMOPEN;

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

                if (verbose) {

                    journal.log(__FUNCTION__,
                                "Could not map memory size.",
                                LogType::EXCEP);
                }

                return_code = return_code + ReturnCode::MEMMAPFAIL;

            }

            new (&tensor_view) MMap<Scalar>(matrix_data,
                                           n_rows,
                                           n_cols);

            return_code = return_code + ReturnCode::MEMMAP;

            if (verbose && vlevel > VLevel::V2) {

                std::string info = "Mapped shared memory at " +
                        mem_path;

                journal.log(__FUNCTION__,
                            info,
                            LogType::INFO);

            }

        }

        template <typename Scalar>
        bool canFitTensor(MMap<Scalar>& mat,
                            int i, int j,
                            Index n_rows2fit,
                            Index n_cols2fit,
                            Journal& journal,
                            bool verbose = true,
                            VLevel vlevel = Journal::VLevel::V0) {

            // Check if the indices (i, j) are within the matrix
            if (i < 0 || i >= mat.rows() || j < 0 || j >= mat.cols()) {

                if (verbose &&
                        vlevel > VLevel::V0) {

                    std::string warn =
                            std::string("Provided indeces are out of bounds wrt memory.");

                    journal.log(__FUNCTION__,
                                 warn,
                             LogType::WARN);

                }

                return false;
            }

            // Check if there's enough space for the submatrix
            if (i + n_rows2fit > mat.rows() ||
                j + n_cols2fit > mat.cols()) {

                if (verbose &&
                        vlevel > VLevel::V0) {

                    std::string warn =
                            std::string("The provided tensor does not fit into memory!");

                    journal.log(__FUNCTION__,
                                 warn,
                                 LogType::WARN);

                }

                return false;
            }

            return true;
        }

        template <typename Scalar>
        bool write(const Tensor<Scalar>& data,
                   MMap<Scalar>& tensor_view,
                   int row, int col,
                   Journal& journal,
                   bool verbose = true,
                   VLevel vlevel = Journal::VLevel::V0) {

            bool success = canFitTensor<Scalar>(tensor_view,
                         row, col,
                         data.rows(), data.cols(),
                         journal,
                         verbose,
                         vlevel);

            if (success) {

                tensor_view.block(row, col,
                              data.rows(),
                              data.cols()) = data;
            }

            return success;

        }

        template <typename Scalar>
        bool read(int row, int col,
                  Tensor<Scalar>& output,
                  MMap<Scalar>& tensor_view,
                  Journal& journal,
                  bool verbose = true,
                  VLevel vlevel = Journal::VLevel::V0) {

            bool success = canFitTensor<Scalar>(tensor_view,
                         row, col,
                         output.rows(), output.cols(),
                         journal,
                         verbose,
                         vlevel);

            if (success) {

                output = tensor_view.block(row, col,
                                           output.rows(),
                                           output.cols());
            }

            return success;

        }

        inline void cleanUpMem(
                        const std::string& mem_path,
                        int& shm_fd,
                        Journal& journal,
                        bool verbose = true,
                        VLevel vlevel = Journal::VLevel::V0,
                        bool unlink = false) {

            // Closing the file descriptor (for this process only)
            ::close(shm_fd);

            if (verbose
                    && vlevel > VLevel::V2) {

                std::string info = "Closed file descriptor for " +
                                    mem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);
            }

            if (unlink) {

                // Unlinking from shared memory data (system-wide)
                // processed which have access to the memory can
                // still access it, but no new process can access it

                shm_unlink(mem_path.c_str());

                if (verbose &&
                        vlevel > VLevel::V2) {

                    std::string info = "Unlinked memory at " +
                                        mem_path;

                    journal.log(__FUNCTION__,
                                 info,
                                 LogType::INFO);
                }
            }

        }

        inline void checkMem(
                    const std::string& mem_path,
                    int& shm_fd,
                    Journal& journal,
                    bool verbose = true,
                    VLevel vlevel = Journal::VLevel::V0) {

            shm_fd = shm_open(mem_path.c_str(),
                               O_RDWR,
                               0);

            if (shm_fd != -1) {
                // Shared memory already exists, so we need to clean it up

                if (verbose &&
                        vlevel > VLevel::V0) {

                    std::string warn = "Shared memory at " + mem_path +
                        " already exists. Clearing it up...";

                    journal.log(__FUNCTION__,
                                 warn,
                                 LogType::WARN);
                }

                cleanUpMem(mem_path,
                         shm_fd,
                         journal,
                         verbose,
                         vlevel);

                if (verbose &&
                        vlevel > VLevel::V0) {

                    std::string warn = "Cleanup Done.";

                    journal.log(__FUNCTION__,
                                 warn,
                                 LogType::WARN);

                }

                if (shm_fd != -1) {
                    // Close the file descriptor opened for checking existence

                    ::close(shm_fd);

                }

            }

        }

        inline int semWait(sem_t* sem,
                        double timeout_seconds) {

            struct timespec timeout;

            clock_gettime(CLOCK_REALTIME, &timeout);

            timeout.tv_sec += timeout_seconds;

            while (true) {

                int result = sem_timedwait(sem,
                                    &timeout);

                if (result == 0) {

                    // Successfully acquired the semaphore.

                    return 0;

                } else if (result == -1 && errno == ETIMEDOUT) {

                    // Timeout occurred.

                    return -1;

                } else if (result == -1 && errno != EINTR) {

                    // Other error occurred (excluding interrupt).

                    return errno;

                }

            }

        }

        inline void closeSem(const std::string& sem_path,
                         sem_t *&sem,
                         Journal& journal,
                         bool verbose = true,
                         VLevel vlevel = Journal::VLevel::V0,
                         bool unlink = false) {

            sem_close(sem); // closes semaphore for the current process

            if (unlink) {

                sem_unlink(sem_path.c_str()); // unlinks semaphore system-wide.
                // Other processes who had it open can still use it, but no new
                // process can access it

                if (verbose &&
                        vlevel > VLevel::V2) {

                    std::string info = std::string("Closed and unlinked semaphore at ") +
                            sem_path;

                    journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);

                }
            }

            if (!unlink) {

                if (verbose &&
                        vlevel > VLevel::V2) {

                    std::string info = std::string("Closed semaphore at ") +
                            sem_path;

                    journal.log(__FUNCTION__,
                         info,
                         LogType::INFO);

                }

            }

        }

        inline void releaseSem(const std::string& sem_path,
                         sem_t *&sem,
                         Journal& journal,
                         bool verbose = true,
                         VLevel vlevel = Journal::VLevel::V0) {

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Releasing semaphore at ") +
                            sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);
            }

            // Release the semaphore
            if (sem_post(sem) == -1) {

                // Handle semaphore release error

                if (verbose) {

                    journal.log(__FUNCTION__,
                                 "Failed to release semaphore at",
                                 LogType::EXCEP);
                }

            }

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Released semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);

            }

        }

        inline void acquireSem(const std::string& sem_path,
                         sem_t*& sem,
                         int n_trials,
                         int& fail_counter,
                         Journal& journal,
                         bool force_reconnection = false,
                         bool verbose = true,
                         VLevel vlevel = Journal::VLevel::V0) {

            if (verbose &&
                    vlevel > VLevel::V2 &&
                    fail_counter == 0) {

                std::string info = std::string("Acquiring semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);

            }

            // Acquire the semaphore
            if (semWait(sem, 1.0) == -1) {

                fail_counter++;

                if (fail_counter > n_trials)
                { // we exceeded the number of allowed trials

                    fail_counter = 0; // reset counter

                    if (verbose) {

                        journal.log(__FUNCTION__,
                                     "Failed to acquire semaphore at",
                                     LogType::EXCEP);

                    }

                }

                if (verbose &&
                        vlevel > VLevel::V0) {

                    std::string warn = std::string("Semaphore acquisition at ") +
                            sem_path +
                            std::string(" failed. Trying to acquire it again...");

                    journal.log(__FUNCTION__,
                                 warn,
                                 LogType::WARN);

                }

                if (force_reconnection) {

                    releaseSem(sem_path,
                            sem,
                            journal,
                            verbose,
                            vlevel); // we try to release it, so that if a previous instance
                    // crashed, we now make the semaphore available for acquisition.
                }

                acquireSem(sem_path,
                            sem,
                            n_trials,
                            fail_counter,
                            journal,
                            force_reconnection,
                            verbose,
                            vlevel); // recursive call. After releaseSems(), this should now work

                if (verbose &&
                    fail_counter > 0 &&
                    vlevel > VLevel::V0) {

                    std::string warn = std::string("Done.");

                    journal.log(__FUNCTION__,
                         warn,
                         LogType::WARN);

                }

            }

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Acquired semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);


            }

            fail_counter = 0; // reset counter

        }

        inline void initSem(const std::string& sem_path,
                         sem_t*& sem,
                         Journal& journal,
                         bool verbose = true,
                         VLevel vlevel = Journal::VLevel::V0) {

            sem = sem_open(sem_path.c_str(),
                                  O_CREAT, S_IRUSR | S_IWUSR,
                                  1);

            if (sem == SEM_FAILED) {
                // Handle semaphore creation error

                if (verbose) {

                    std::string error = std::string("Failed to open semaphore");

                    journal.log(__FUNCTION__,
                        error,
                        LogType::EXCEP);

                }

            }
            else {

                if (verbose &&
                    vlevel > VLevel::V2) {

                    std::string info = std::string("Opened semaphore at ") +
                            sem_path;

                    journal.log(__FUNCTION__,
                        info,
                        LogType::INFO);
                }
            }

        }
    }

}

#endif // MEMUTILS_HPP
