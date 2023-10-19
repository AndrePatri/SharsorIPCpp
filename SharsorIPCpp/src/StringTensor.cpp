#include <SharsorIPCpp/StringTensor.hpp>

namespace SharsorIPCpp {

    template <typename ShMemType>
    StringTensor<ShMemType>::StringTensor(std::string basename,
                                       std::string name_space,
                                       bool verbose,
                                       VLevel vlevel)
    : sh_mem(Server<int>(1, 1,
                         "SharsorDouble", "name_space",
                        true,
                        VLevel::V3,
                        true))  {

    }

    template <typename ShMemType>
    StringTensor<ShMemType>::StringTensor(int lenght,
                                       std::string basename,
                                       std::string name_space,
                                       bool verbose,
                                       VLevel vlevel,
                                       bool force_reconnection)
        : sh_mem(Server<int>(1, 1,
                        "SharsorDouble", "name_space",
                        true,
                        VLevel::V3,
                        true)) {
    }

    template <typename ShMemType>
    StringTensor<ShMemType>::~StringTensor() {


    }

    template <typename ShMemType>
    void StringTensor<ShMemType>::write(const std::vector<std::string>& vec,
                                   int index) {


    }

    template <typename ShMemType>
    void StringTensor<ShMemType>::read(const std::vector<std::string>& vec,
                                   int index) {


    }

    // explicit instantiations (increases library size and compilation time)

//    template class StringTensor<int>;
    template class StringTensor<Server<int>>;
    template class StringTensor<Client<int>>;

    template <>
    Server<int> StringTensor<Server<int>>::_initServer(int length,
                                                       std::string basename,
                                                       std::string name_space,
                                                       VLevel vlevel) {

        return Server<int>(1, 1,
                           "SharsorDouble",
                           "name_space",
                          true,
                          VLevel::V3,
                          true);

    }

    template <>
    Client<int> StringTensor<Client<int>>::_initClient(std::string basename,
                                                       std::string name_space,
                                                       VLevel vlevel) {

        return Client<int>("SharsorDouble",
                           "name_space",
                          true,
                          VLevel::V3);

    }
}
