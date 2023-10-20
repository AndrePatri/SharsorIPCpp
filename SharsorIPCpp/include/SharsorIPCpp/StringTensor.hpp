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

        // Wrapper on top of Server and Client to also share
        // tensor (list) of strings. The idea is to use a
        // shared tensor of ints with the following properties:
        // each string in the list will be encoded along the columns of
        // the tensor. Given an input string, the string is first split
        // into chunks of size equal to size_of(int), which is typically 4 bytes or 32 bits.
        // This means, for most systems, we can represent each chunk of a string as a sequence
        // of up to 4 characters (UTF-8 encoded). Each chunck can then distributed
        // along the column of the tensor, at the corresponding column index.
        // Strings are encoded using UTF8 and then decoded with the same logic.

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

            bool write(const std::vector<std::string>& vec,
                       int index = 0);

            bool write(const std::string& str,
                       int index = 0);

            bool read(std::vector<std::string>& vec,
                      int index = 0);

            bool read(std::string& str,
                      int index = 0);

            void close();

            bool isRunning();

            int length = -1; // string tensor length

        private:

            bool _running = false;

            // assuming maximum characters in a string are max_chars
            static constexpr int _max_chars = 1024;

            // max number of rows needed
            static constexpr int _n_rows = _max_chars / sizeof(int);

            int _tmp_value = 0; // tmp val to avoid dyn allocations

            Tensor<int> _buffer = Tensor<int>(_n_rows, 1); // to avoid dyn. allocations

            ShMemType _sh_mem;

            // Helper methods to initialize _sh_mem
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

            bool _fits(const std::vector<std::string>& vec,
                       int index);

            void _encode_str(const std::string& str,
                             int col_index);

            void _decode_str(std::string& str,
                             int col_index);

            bool _encode_vec(const std::vector<std::string>& vec,
                             int col_index);

            bool _decode_vec(std::vector<std::string>& vec,
                           int col_index);

        };

}

#endif // STRINGTENSOR_HPP
