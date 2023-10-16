#ifndef JOURNAL_HPP
#define JOURNAL_HPP

#include <iostream>
#include <cstdio>
#include <string>
#include <cstring> // for std::strerror()

namespace SharsorIPCpp{

    namespace Colors {
        const std::string kRed = "\033[0;31m";
        const std::string kBoldRed = "\033[1;31m";
        const std::string kGreen = "\033[0;32m";
        const std::string kBoldGreen = "\033[1;32m";
        const std::string kYellow = "\033[0;33m";
        const std::string kBoldYellow = "\033[1;33m";
        const std::string kBlue = "\033[0;34m";
        const std::string kBoldBlue = "\033[1;34m";
        const std::string kMagenta = "\033[0;35m";
        const std::string kBoldMagenta = "\033[1;35m";
        const std::string kCyan = "\033[0;36m";
        const std::string kBoldCyan = "\033[1;36m";
        const std::string kEndl = "\033[0m";
    };

    class Journal {

        public:

            enum class LogType {WARN,
                                EXCEP,
                                INFO,
                                STAT};

            enum class VLevel {V0, // nothing except errors
                    V1, // only warnings
                    V2, // warning + statistics
                    V3}; // warning + statistics + additional info

            Journal(const std::string& classname) :
                _classname(classname) {
            }

            void log(const std::string& methodname,
                     const std::string& message,
                     LogType log_type) {

                // Map LogType to corresponding string
                const char* logTypeStr = logTypeToString(log_type);

                if (log_type == LogType::EXCEP){

                    std::string exception =
                            Colors::kBoldRed +
                            std::string("[") +
                            _classname +
                            std::string("]") +
                            std::string("[") +
                            methodname +
                            std::string("]") +
                            std::string("[") +
                            logTypeStr +
                            std::string("]: ") +
                            message +
                            std::string(", error code ") +
                            std::string(std::strerror(errno)) +
                            Colors::kEndl;

                    throw std::runtime_error(exception);

                }

                if (log_type == LogType::WARN){

                    std::printf("%s[%s][%s][%s]: %s%s\n",
                            Colors::kBoldYellow.c_str(),
                            _classname.c_str(),
                            methodname.c_str(),
                            logTypeStr,
                            message.c_str(),
                            Colors::kEndl.c_str());

                }

                if (log_type == LogType::INFO){

                    std::printf("%s[%s][%s][%s]: %s%s\n",
                            Colors::kBoldGreen.c_str(),
                            _classname.c_str(),
                            methodname.c_str(),
                            logTypeStr,
                            message.c_str(),
                            Colors::kEndl.c_str());

                }

                if (log_type == LogType::STAT){

                    std::printf("%s[%s][%s][%s]: %s%s\n",
                            Colors::kBoldBlue.c_str(),
                            _classname.c_str(),
                            methodname.c_str(),
                            logTypeStr,
                            message.c_str(),
                            Colors::kEndl.c_str());

                }


            }

        private:

            std::string _classname;

            // Helper function to convert LogType to string
            const char* logTypeToString(LogType log_type) {
                switch (log_type) {
                    case LogType::WARN:
                        return "WARN";
                    case LogType::EXCEP:
                        return "EXCEP";
                    case LogType::INFO:
                        return "INFO";
                    case LogType::STAT:
                        return "STAT";
                    default:
                        return "~";
                }
            }
    };

}
#endif // JOURNAL_HPP
