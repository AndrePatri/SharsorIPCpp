#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include <SharsorIPCpp/Journal.hpp>

#include <random>

namespace SharsorIPCpp {

    void check_comp_type(Journal& journal)
    {
        std::string message;

        #ifdef NDEBUG

            #ifdef _RELWITHDEBINFO

                message = std::string("SharsorIPCpp was compiled in RelWithDebInfo mode. ") +
                        std::string("For meaninful results, you should compile it in Release mode.\n");

                journal.log("check_comp_type",
                            message,
                            Journal::LogType::WARN);

            #else

                message = std::string("SharsorIPCpp was compiled in Release mode. ") +
                    std::string("This is good and will ensure meaningful benchmarking results.\n");

                journal.log("check_comp_type",
                            message,
                            Journal::LogType::STAT);

            #endif

        #else

            message = std::string("SharsorIPCpp was compiled in Debug mode. ") +
                std::string("For meaninful results, you should compile it in Release mode.\n");

            journal.log("check_comp_type",
                        message,
                        Journal::LogType::WARN);

        #endif

    }

    std::string random_string(size_t length) {

        static const char alphabet[] = "abcdefghijklmnopqrst)(/)/£0430*é°è{}]54309582'2'uvwxyz";  // Add more characters if needed

        static std::default_random_engine rng(std::random_device{}());
        static std::uniform_int_distribution<> dist(0, sizeof(alphabet) - 2);  // -2 because sizeof(alphabet) includes the null-terminator

        std::string result;
        result.reserve(length);

        for(size_t i = 0; i < length; ++i) {
            result.push_back(alphabet[dist(rng)]);
        }

        return result;
    }

    int random_int(int n) {

        static std::default_random_engine rng(std::random_device{}());

        std::uniform_int_distribution<> dist(0, n);

        return dist(rng);
    }

}


#endif // TEST_UTILS_HPP
