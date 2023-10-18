#ifndef TIMEUTILS_HPP
#define TIMEUTILS_HPP

#include <thread>
#include <chrono>
#include <ctime>

namespace SharsorIPCpp {

    namespace TimeUtils {

        class PerfSleep {

            public:

                PerfSleep();

                timespec _req, _rem;

                static void thread_nsleep(int nsecs)
                {
                    std::this_thread::sleep_for(std::chrono::nanoseconds(nsecs));
                }

                static void thread_usleep(int usecs)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(usecs));
                }

                static void thread_msleep(int msecs)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
                }

                static void thread_sleep(int secs)
                {
                    std::this_thread::sleep_for(std::chrono::seconds(secs));
                }

                void clock_nsleep(int nsecs)
                {
                    _req.tv_nsec = nsecs;

                    clock_nanosleep(CLOCK_REALTIME, 0, &_req, &_rem);
                }

                void clock_usleep(int usecs)
                {
                    _req.tv_sec = usecs / 1000000;  // Convert microseconds to seconds
                    _req.tv_nsec = (usecs % 1000000) * 1000;  // Convert the remainder to nanoseconds

                    clock_nanosleep(CLOCK_REALTIME, 0, &_req, &_rem);
                }

                void clock_msleep(int msecs)
                {
                    _req.tv_sec = msecs / 1000;  // Convert milliseconds to seconds
                    _req.tv_nsec = (msecs % 1000) * 1000000;  // Convert the remainder to nanoseconds

                    clock_nanosleep(CLOCK_REALTIME, 0, &_req, &_rem);
                }

                void clock_sleep(int secs)
                {
                    _req.tv_sec = secs;
                    _req.tv_nsec = 0;

                    clock_nanosleep(CLOCK_REALTIME, 0, &_req, &_rem);
                }
        };

    }


}
#endif // TIMEUTILS_HPP
