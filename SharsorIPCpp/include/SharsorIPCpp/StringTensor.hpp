#ifndef STRINGTENSOR_HPP
#define STRINGTENSOR_HPP

#include <vector>
#include <string>

#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Server.hpp>

#include <SharsorIPCpp/Journal.hpp>

namespace SharsorIPCpp {

        using StrServer = Server<int>;
        using StrClient = Client<int>;

        template <typename ShMemType>
        class StringTensor {

        public:

            typedef std::weak_ptr<StringTensor> WeakPtr;
            typedef std::shared_ptr<StringTensor> Ptr;
            typedef std::unique_ptr<StringTensor> UniquePtr;

            StringTensor(std::string basename = "MySharedMemory",
                         std::string name_space = "",
                         bool verbose = false,
                         VLevel vlevel = VLevel::V0); // used when client

            StringTensor(int lenght,
                         std::string basename = "MySharedMemory",
                         std::string name_space = "",
                         bool verbose = false,
                         VLevel vlevel = VLevel::V0,
                         bool force_reconnection = false); // used when server

            ~StringTensor();

            void run();

            void write(const std::vector<std::string>& vec,
                       int index = 0);

            void read(const std::vector<std::string>& vec,
                      int index = 0);

            void close();

            int length = -1;

        private:

            ShMemType sh_mem;

            // Helper methods to initialize sh_mem
            ShMemType _initServer(int length,
                                  std::string basename,
                                  std::string name_space,
                                  bool verbose,
                                  VLevel vlevel,
                                  bool force_reconnection);

            ShMemType _initClient(std::string basename,
                                  std::string name_space,
                                  bool verbose,
                                  VLevel vlevel);

        };

}

#endif // STRINGTENSOR_HPP
