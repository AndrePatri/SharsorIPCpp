#ifndef STRINGTENSOR_HPP
#define STRINGTENSOR_HPP

#include <vector>
#include <string>

#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Server.hpp>

#include <SharsorIPCpp/Journal.hpp>

namespace SharsorIPCpp {

        template <typename ShMemType>
        class StringTensor {

        public:

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

            void write(const std::vector<std::string>& vec,
                       int index = 0);

            void read(const std::vector<std::string>& vec,
                      int index = 0);

            int length = -1;

        private:

            Server<int> sh_mem;

            // Helper methods to initialize sh_mem
            ShMemType _initServer(int length,
                                  std::string basename,
                                  std::string name_space,
                                  VLevel vlevel);

            ShMemType _initClient(std::string basename,
                                  std::string name_space,
                                  VLevel vlevel);

        };

}

#endif // STRINGTENSOR_HPP
