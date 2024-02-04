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

#include <SharsorIPCpp/StringTensor.hpp>

namespace SharsorIPCpp {

    // template specializations

    // constructors helpers
    template <>
    StrClient StringTensor<StrClient>::_initClient(std::string basename,
                                                   std::string name_space,
                                                   bool verbose,
                                                   VLevel vlevel) {

        return StrClient(basename,
                  name_space,
                  verbose,
                  vlevel);

    }

    template <>
    StrServer StringTensor<StrServer>::_initServer(int length,
                                                   std::string basename,
                                                   std::string name_space,
                                                   bool verbose,
                                                   VLevel vlevel,
                                                   bool force_reconnection) {

        return StrServer(_n_rows, length,
                        basename,
                        name_space,
                        verbose,
                        vlevel,
                        force_reconnection);

    }

    // constructor specialization for Client
    template <>
    StringTensor<StrClient>::StringTensor(std::string basename,
                                           std::string name_space,
                                           bool verbose,
                                           VLevel vlevel)
    : _sh_mem(_initClient(basename, name_space,
                         verbose, vlevel)) {


    }

    // constructor specialization for Server
    template <>
    StringTensor<StrServer>::StringTensor(int length,
                                           std::string basename,
                                           std::string name_space,
                                           bool verbose,
                                           VLevel vlevel,
                                           bool force_reconnection)
    : _sh_mem(_initServer(length,
                         basename, name_space,
                         verbose, vlevel,
                         force_reconnection)),
      _length(length),
      _buffer(Tensor<int>(_n_rows, length)), // we can initialize buffer
      _is_server(true){ 

    }

    template <>
    void StringTensor<StrClient>::run() {

        if (!_running) {

            _sh_mem.attach();

            _length =_sh_mem.getNCols(); // getting from
            // client (client gets this from server)

            _buffer = Tensor<int>(_n_rows, _length); // we can now initialize the buffer

            _running = true;
        }

    }

    template <>
    void StringTensor<StrServer>::run() {

        if (!_running) {

            _sh_mem.run();

            _running = true;
        }

    }

    // class specialization
    template class StringTensor<StrServer>;
    template class StringTensor<StrClient>;


    template <typename ShMemType>
    StringTensor<ShMemType>::~StringTensor() {

        _sh_mem.close();

    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::isServer() {
        
        return _is_server;
    }
    
    template <>
    int StringTensor<StrServer>::getNClients() {

        return _sh_mem.getNClients();

    }

    template <>
    int StringTensor<StrClient>::getNClients() {

        return -1;

    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::read(std::vector<std::string>& vec,
                                       int col_index) {

        // (&& guarantees short-circuit evaluation, i.e.
        // ordered evaluation)

        return isRunning() &&
               _sh_mem.read(_buffer.block(0, col_index,
                                                _n_rows, vec.size()), 0, col_index) && // just update the block
               _decode_vec(vec, col_index);

    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::read(std::string& str,
                                       int col_index) {


        if (!isRunning() ||
            col_index < 0 ||
            col_index >= _length ||
            !_sh_mem.read(_buffer.block(0, col_index,
                                              _n_rows, 1), 0, col_index) // just update the right column
                                ) {

            return false;
        }

        _decode_str(str, col_index);

        return true;


    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::write(const std::vector<std::string>& vec,
                                        int col_index) {

        return isRunning() &&
               _encode_vec(vec, col_index) &&
               _sh_mem.write(_buffer.block(0, col_index,
                                                  _n_rows, vec.size()),
                                    0, col_index);

    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::write(const std::string& str,
                                        int col_index) {

        if (!isRunning() ||
            col_index < 0 ||
            col_index >= _length) {

            return false;
        }

        _encode_str(str, col_index);

        return  _sh_mem.write(_buffer.block(0, col_index,
                                                  _n_rows, 1),
                                    0, col_index);

    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::isRunning() {

        return _running;

    }

    template <typename ShMemType>
    int StringTensor<ShMemType>::getLength() {

        return _length;

    }

    template <typename ShMemType>
    void StringTensor<ShMemType>::close() {

        _sh_mem.close();

    }

    template <typename ShMemType>
    Tensor<int> StringTensor<ShMemType>::get_raw_buffer() {
        
        // returns a copy (user must not be allowed to modify
        // the buffer in unintended ways)

        return _buffer;

    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::_encode_vec(const std::vector<std::string>& vec, int col_index) {

        if (!_fits(vec, col_index)) {

            return false;

        }

        for (const auto& str : vec) { // for each string

            _encode_str(str, col_index);

            ++col_index; // go to next column (i.e. string)
        }

        return true;
    }

    template <typename ShMemType>
    void StringTensor<ShMemType>::_encode_str(const std::string& str,
                                       int col_index) {

        _buffer.col(col_index).setZero(); // reset buffer

        for (size_t i = 0, row = 0;
             i < str.size();
             i += sizeof(int), ++row) { // increment row counter
            // and move through the string by sizeof(int)

            _tmp_value = 0;

            for (size_t j = 0;
                 j < sizeof(int) && i + j < str.size(); // move within the chunk up to string size
                 ++j) {

                _tmp_value |= // UTF8 encoding
                       (static_cast<int>(static_cast<unsigned char>(str[i + j])) << (j * 8));

            }

            _buffer(row, col_index) = _tmp_value; // write to tmp buffer

        }

    }

    template <typename ShMemType>
    void StringTensor<ShMemType>::_decode_str(std::string& str,
                                       int col_index) {

        str.clear();

        for (int row = 0; row < _n_rows; ++row) { // for each chunk

            _tmp_value = _buffer(row, col_index);

            for (size_t j = 0; j < sizeof(int); ++j) {

                char c = (_tmp_value >> (j * 8)) & 0xFF; // decode to UTF8

                if (c == 0) break;

                str.push_back(c); // add to string
            }

            if (str.size() % sizeof(int) != 0) break; // exit if out of range

        }

    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::_decode_vec(std::vector<std::string>& vec,
                                       int col_index) {

        if (!_fits(vec, col_index)) {

            return false;
        }

        for (auto& str : vec) { // for each string

            _decode_str(str, col_index);

            ++col_index; // go to next string (i.e. column)
        }

        return true;
    }

    template <typename ShMemType>
    bool StringTensor<ShMemType>::_fits(const std::vector<std::string>& vec,
                                       int index) {

        if (index + vec.size() > _length) {

            return false;
        }

        return true;

    }

}
