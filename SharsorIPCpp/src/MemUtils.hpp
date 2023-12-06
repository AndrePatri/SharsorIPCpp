// Copyright (C) 2023  Andrea Patrizi (AndrePatri)
// 
// This file is part of SharsorIPCpp and distributed under the General Public License version 2 license.
// 
// SharsorIPCpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// SharsorIPCpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with SharsorIPCpp.  If not, see <http://www.gnu.org/licenses/>.
// 
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
#include <SharsorIPCpp/Helpers.hpp>

namespace SharsorIPCpp{

    namespace MemUtils{
        
        using VLevel = Journal::VLevel;

        using LogType = Journal::LogType;

        // Define a type trait to check if a given type is a valid DType
        template <typename Scalar>
        struct IsValidDType {
            static constexpr bool value =
                std::is_same<Scalar, typename DTypeToCppType<DType::Float>::type>::value ||
                std::is_same<Scalar, typename DTypeToCppType<DType::Double>::type>::value ||
                std::is_same<Scalar, typename DTypeToCppType<DType::Int>::type>::value ||
                std::is_same<Scalar, typename DTypeToCppType<DType::Bool>::type>::value;
        };

        inline std::string getLayoutName(int layout) {

            switch(layout) {

                case RowMajor:

                    return "SharsorIPCpp::RowMajor";

                case ColMajor:

                    return "SharsorIPCpp::ColMajor";

                default:

                    return "Unknown memory layout";
            }
        }

        template <typename Scalar,
                  int Layout = MemLayoutDefault>
        void initMem(
            std::size_t n_rows,
            std::size_t n_cols,
            const std::string& mem_path,
            int& shm_fd,
            MMap<Scalar, Layout>& tensor_view,
            Journal& journal,
            ReturnCode& return_code,
            bool verbose = true,
            VLevel vlevel = Journal::VLevel::V0
            )
        {

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

                return;

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

                return;

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

                return;

            }

            new (&tensor_view) MMap<Scalar, Layout>(matrix_data,
                                           n_rows,
                                           n_cols); // contiguous memory
            // (no need to specify strides)

            return_code = return_code + ReturnCode::MEMMAP;

            if (verbose && vlevel > VLevel::V2) {

                std::string info = "Mapped shared memory at " +
                        mem_path;

                journal.log(__FUNCTION__,
                            info,
                            LogType::INFO);

            }

        }

        template <typename Scalar,
                  int Layout = MemLayoutDefault>
        bool write(const TRef<Scalar, Layout> data, // eigen reference (works also with blocks)
                   MMap<Scalar, Layout>& tensor_view,
                   int row, int col,
                   Journal& journal,
                   ReturnCode& return_code,
                   bool verbose = true,
                   VLevel vlevel = Journal::VLevel::V0) {

            bool success = helpers::canFitTensor(
                        tensor_view.rows(),
                        tensor_view.cols(),
                        row, col,
                        data.rows(), data.cols(),
                        journal,
                        return_code,
                        verbose,
                        vlevel);

            if (success) {

                tensor_view.block(row, col,
                              data.rows(),
                              data.cols()) = data;
            }

            if (!success) {

                return_code = return_code + ReturnCode::WRITEFAIL;
            }

            return success;

        }

        template <typename Scalar,
                  int Layout = MemLayoutDefault>
        bool write(const TensorView<Scalar, Layout>& data,
                   MMap<Scalar, Layout>& tensor_view,
                   int row, int col,
                   Journal& journal,
                   ReturnCode& return_code,
                   bool verbose = true,
                   VLevel vlevel = Journal::VLevel::V0) {

            bool success = helpers::canFitTensor(
                         tensor_view.rows(),
                         tensor_view.cols(),
                         row, col,
                         data.rows(), data.cols(),
                         journal,
                         return_code,
                         verbose,
                         vlevel);

            if (success) {

                tensor_view.block(row, col,
                              data.rows(),
                              data.cols()) = data;
            }

            if (!success) {

                return_code = return_code + ReturnCode::WRITEFAIL;
            }

            return success;

        }

        template <typename Scalar,
                  int Layout = MemLayoutDefault>
        bool read(int row, int col,
                  TRef<Scalar, Layout> output,
                  const MMap<Scalar, Layout>& tensor_view,
                  Journal& journal,
                  ReturnCode& return_code,
                  bool verbose = true,
                  VLevel vlevel = Journal::VLevel::V0) {

            bool success = helpers::canFitTensor(
                        tensor_view.rows(),
                        tensor_view.cols(),
                        row, col,
                        output.rows(), output.cols(),
                        journal,
                        return_code,
                        verbose,
                        vlevel);

            if (success) {

                output = tensor_view.block(row, col,
                                           output.rows(),
                                           output.cols());
            }

            if (!success) {

                return_code = return_code + ReturnCode::READFAIL;
            }

            return success;

        }

        template <typename Scalar,
                  int Layout = MemLayoutDefault>
        bool read(int row, int col,
                  TensorView<Scalar, Layout>& output,
                  const MMap<Scalar, Layout>& tensor_view,
                  Journal& journal,
                  ReturnCode& return_code,
                  bool verbose = true,
                  VLevel vlevel = Journal::VLevel::V0) {

            // copy data pointed from tensor_view
            // into the matrix output points
            bool success = helpers::canFitTensor(
                        tensor_view.rows(),
                        tensor_view.cols(),
                        row, col,
                        output.rows(), output.cols(),
                        journal,
                        return_code,
                        verbose,
                        vlevel);

            if (success) {

                output  = tensor_view.block(row, col,
                                           output.rows(),
                                           output.cols());
            }

            if (!success) {

                return_code = return_code + ReturnCode::READFAIL;
            }

            return success;

        }

        inline void failWithCode(ReturnCode fail_code,
                                 Journal journal) {

            std::string error = std::string("Failed with error code: ") +
                    toString(fail_code);

            // we throw an exception
            journal.log(__FUNCTION__,
                         error,
                         LogType::EXCEP,
                         true);

        }

        inline void cleanUpMem(
                        const std::string& mem_path,
                        int& shm_fd,
                        Journal& journal,
                        ReturnCode& return_code,
                        bool verbose = true,
                        VLevel vlevel = Journal::VLevel::V0,
                        bool unlink = false) {

            // Closing the file descriptor (for this process only)
            ::close(shm_fd);

            return_code = return_code + ReturnCode::MEMFDCLOSED;

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

                return_code = return_code + ReturnCode::MEMUNLINK;
            }

        }

        inline void checkMem(
                    const std::string& mem_path,
                    int& shm_fd,
                    Journal& journal,
                    ReturnCode& return_code,
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

                return_code = return_code + ReturnCode::MEMEXISTS;

                cleanUpMem(mem_path,
                         shm_fd,
                         journal,
                         return_code,
                         verbose,
                         vlevel);

                return_code = return_code + ReturnCode::MEMCLEAN;

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

                    return_code = return_code + ReturnCode::MEMFDCLOSED;

                }

            }

        }

        inline int semWait(sem_t* sem,
                        double timeout_seconds,
                        ReturnCode& return_code) {

            struct timespec timeout;

            clock_gettime(CLOCK_REALTIME, &timeout);

            timeout.tv_sec += timeout_seconds;

            while (true) {

                int result = sem_timedwait(sem,
                                    &timeout);

                if (result == 0) {

                    // Successfully acquired the semaphore.

                    return_code = return_code + ReturnCode::SEMACQ;

                    return 0;

                } else if (result == -1 && errno == ETIMEDOUT) {

                    return_code = return_code + ReturnCode::SEMACQTIMEOUT;

                    // Timeout occurred.

                    return -1;

                } else if (result == -1 && errno != EINTR) {

                    // Other error occurred (excluding interrupt).

                    return_code = return_code + ReturnCode::OTHER;

                    return errno;

                }

            }

        }

        inline void closeSem(const std::string& sem_path,
                         sem_t *&sem,
                         Journal& journal,
                         ReturnCode& return_code,
                         bool verbose = true,
                         VLevel vlevel = Journal::VLevel::V0,
                         bool unlink = false) {

            sem_close(sem); // closes semaphore for the current process

            return_code = return_code + ReturnCode::SEMCLOSE;

            if (unlink) {

                sem_unlink(sem_path.c_str()); // unlinks semaphore system-wide.
                // Other processes who had it open can still use it, but no new
                // process can access it

                return_code = return_code + ReturnCode::SEMUNLINK;

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
                         ReturnCode& return_code,
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

                return_code = return_code + ReturnCode::SEMRELFAIL;

                return;

            }

            return_code = return_code + ReturnCode::SEMREL;

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Released semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);

            }

        }

        inline void acquireSemTry(const std::string& sem_path,
                         sem_t*& sem,
                         Journal& journal,
                         ReturnCode& return_code,
                         bool verbose = true,
                         VLevel vlevel = Journal::VLevel::V0) {

            // this is nonblocking --> suitable for RT systems

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Acquiring semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);

            }

            // try to acquire the semaphore
            if (sem_trywait(sem) == -1) {

                // failed --> return

                if (verbose) {

                    std::string error = std::string("Failed to acquire semaphore at ") +
                                    sem_path;
                    journal.log(__FUNCTION__,
                                 error,
                                 LogType::EXCEP);

                }

                return_code = return_code + ReturnCode::SEMACQFAIL;

                return;

            }

            return_code = return_code + ReturnCode::SEMACQ;

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Acquired semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);


            }

        }

        inline void acquireSemWait(const std::string& sem_path,
                        sem_t*& sem,
                        int n_trials,
                        int& fail_counter,
                        Journal& journal,
                        ReturnCode& return_code,
                        float wait_for = 0.2, // [s]
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
            if (semWait(sem, wait_for,
                        return_code) == -1) {

                fail_counter++;

                return_code = return_code + ReturnCode::SEMACQRETRY;

                if (fail_counter > n_trials)
                { // we exceeded the number of allowed trials

                    fail_counter = 0; // reset counter

                    if (verbose) {

                        journal.log(__FUNCTION__,
                                     "Failed to acquire semaphore at",
                                     LogType::EXCEP);

                    }

                    return_code = return_code + ReturnCode::SEMACQFAIL;

                    return;

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
                            return_code,
                            verbose,
                            vlevel); // we try to release it, so that if a previous instance
                    // crashed, we now make the semaphore available for acquisition.

                    return_code = return_code + ReturnCode::SEMREL;
                }

                acquireSemWait(sem_path,
                            sem,
                            n_trials,
                            fail_counter,
                            journal,
                            return_code,
                            wait_for,
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

                return_code = return_code + ReturnCode::SEMACQ;

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
                         ReturnCode& return_code,
                         bool verbose = true,
                         VLevel vlevel = Journal::VLevel::V0) {

            sem = sem_open(sem_path.c_str(),
                                  O_CREAT, S_IRUSR | S_IWUSR,
                                  1);

            if (sem == SEM_FAILED) {
                // Handle semaphore creation error

                return_code = return_code + ReturnCode::SEMOPENFAIL;

                if (verbose) {

                    std::string error = std::string("Failed to open semaphore at ") +
                            sem_path;

                    journal.log(__FUNCTION__,
                        error,
                        LogType::EXCEP);

                }

                return;

            }
            else {

                return_code = return_code + ReturnCode::SEMOPEN;

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
