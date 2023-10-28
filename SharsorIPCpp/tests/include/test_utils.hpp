#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include <SharsorIPCpp/Journal.hpp>
#include <SharsorIPCpp/DTypes.hpp>

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

    template <typename T, int Layout>
    struct Thresholds;

    template <>
    struct Thresholds<bool, SharsorIPCpp::RowMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 1500;
        static constexpr double READ_TV_AVRG_THRESH = 1500;
        static constexpr double WRITE_T_AVRG_THRESH = 1500;

    };

    template <>
    struct Thresholds<bool, SharsorIPCpp::ColMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 1500;
        static constexpr double READ_TV_AVRG_THRESH = 1500;
        static constexpr double WRITE_T_AVRG_THRESH = 1500;

    };

    template <>
    struct Thresholds<int, SharsorIPCpp::RowMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 2300;
        static constexpr double READ_TV_AVRG_THRESH = 3000;
        static constexpr double WRITE_T_AVRG_THRESH = 2300;

    };

    template <>
    struct Thresholds<int, SharsorIPCpp::ColMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 2300;
        static constexpr double READ_TV_AVRG_THRESH = 2500;
        static constexpr double WRITE_T_AVRG_THRESH = 2300;

    };

    template <>
    struct Thresholds<float, SharsorIPCpp::RowMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 2500;
        static constexpr double READ_TV_AVRG_THRESH = 3000;
        static constexpr double WRITE_T_AVRG_THRESH = 2500;

    };

    template <>
    struct Thresholds<float, SharsorIPCpp::ColMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 2500;
        static constexpr double READ_TV_AVRG_THRESH = 3000;
        static constexpr double WRITE_T_AVRG_THRESH = 2500;

    };

    template <>
    struct Thresholds<double, SharsorIPCpp::RowMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 5000;
        static constexpr double READ_TV_AVRG_THRESH = 5000;
        static constexpr double WRITE_T_AVRG_THRESH = 5000;

    };

    template <>
    struct Thresholds<double, SharsorIPCpp::ColMajor> {
        static constexpr double READ_T_MAX_THRESH = 10000000;
        static constexpr double READ_TV_MAX_THRESH = 10000000;
        static constexpr double WRITE_T_MAX_THRESH = 10000000;
        static constexpr double READ_T_AVRG_THRESH = 5000;
        static constexpr double READ_TV_AVRG_THRESH = 5000;
        static constexpr double WRITE_T_AVRG_THRESH = 5000;

    };
}


#endif // TEST_UTILS_HPP
