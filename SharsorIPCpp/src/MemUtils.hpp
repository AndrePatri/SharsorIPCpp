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

        template <typename Scalar>
        void write(const Tensor<Scalar>& data,
                   MMap<Scalar>& tensor_view,
                   Journal& journal) {

            if(data.rows() != tensor_view.rows()
                    || data.cols() !=  tensor_view.cols()) {

                std::string error =
                        std::string("Data dimension mismatch. ") +
                        std::string("Expected a tensor of size ") +
                        std::to_string(tensor_view.rows()) + std::string("x") +
                        std::to_string(tensor_view.cols()) +
                        std::string(", but provided tensor is ") +
                        std::to_string(data.rows()) + std::string("x") +
                        std::to_string(data.cols());

                journal.log(__FUNCTION__,
                             error,
                             LogType::EXCEP);

            }

            tensor_view.block(0, 0,
                              tensor_view.rows(),
                              tensor_view.cols()) = data;

        }

        template <typename Scalar>
        void cleanUpMem(
                const std::string& mem_path,
                int& shm_fd,
                Journal& journal,
                bool verbose = true,
                VLevel vlevel = Journal::VLevel::V0) {

            // Unlinking from shared memory data
            shm_unlink(mem_path.c_str());

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = "Unlinked from memory at " +
                                    mem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);
            }

            // Closing the file descriptor
            ::close(shm_fd);

            if (verbose
                    && vlevel > VLevel::V2) {

                std::string info = "Closed file descriptor for " +
                                    mem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);
            }

        }

        template <typename Scalar>
        void checkMem(
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

                cleanUpMem<Scalar>(mem_path,
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

        template <typename Scalar>
        int semWait(sem_t* sem,
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

        template <typename Scalar>
        void closeSem(const std::string& sem_path,
                     sem_t *&sem,
                     Journal& journal,
                     bool verbose = true,
                     VLevel vlevel = Journal::VLevel::V0) {

            sem_close(sem); // close semaphore

            sem_unlink(sem_path.c_str()); // unlink semaphore

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Closed and unlinked server semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                     info,
                     LogType::INFO);

            }

        }

        template <typename Scalar>
        void releaseSem(const std::string& sem_path,
                     sem_t *&sem,
                     Journal& journal,
                     bool verbose = true,
                     VLevel vlevel = Journal::VLevel::V0) {

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Releasing server semaphore at ") +
                            sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);
            }

            // Release the semaphore
            if (sem_post(sem) == -1) {

                // Handle semaphore release error

                journal.log(__FUNCTION__,
                             "Failed to release semaphore at",
                             LogType::EXCEP);
            }

            if (verbose &&
                    vlevel > VLevel::V2) {

                std::string info = std::string("Released server semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);

            }

        }

        template <typename Scalar>
        void acquireSem(const std::string& sem_path,
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

                std::string info = std::string("Acquiring server semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);

            }

            // Acquire the semaphore
            if (semWait<Scalar>(sem, 1.0) == -1) {

                fail_counter++;

                if (fail_counter > n_trials)
                { // we exceeded the number of allowed trials

                    fail_counter = 0; // reset counter

                    journal.log(__FUNCTION__,
                                 "Failed to acquire semaphore at",
                                 LogType::EXCEP);

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

                    releaseSem<Scalar>(sem_path,
                            sem,
                            journal,
                            verbose,
                            vlevel); // we try to release it, so that if a previous server
                    // crashed, we now make the semaphore available for acquisition.
                }

                acquireSem<Scalar>(sem_path,
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

                std::string info = std::string("Acquired server semaphore at ") +
                        sem_path;

                journal.log(__FUNCTION__,
                             info,
                             LogType::INFO);


            }

            fail_counter = 0; // reset counter

        }

        template <typename Scalar>
        void initSem(const std::string& sem_path,
                     sem_t*& sem,
                     Journal& journal,
                     bool verbose = true,
                     VLevel vlevel = Journal::VLevel::V0) {

            sem = sem_open(sem_path.c_str(),
                                  O_CREAT, S_IRUSR | S_IWUSR,
                                  1);

            if (sem == SEM_FAILED) {
                // Handle semaphore creation error

                std::string error = std::string("Failed to open semaphore");

                journal.log(__FUNCTION__,
                    error,
                    LogType::EXCEP);
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
