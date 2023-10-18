#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include <SharsorIPCpp/Journal.hpp>

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

}


#endif // TEST_UTILS_HPP
