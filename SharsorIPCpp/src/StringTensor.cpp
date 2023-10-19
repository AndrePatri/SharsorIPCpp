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

        return StrServer(1, length,
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
    : sh_mem(_initClient(basename, name_space,
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
    : sh_mem(_initServer(length,
                         basename, name_space,
                         verbose, vlevel,
                         force_reconnection)) {

    }

    template <>
    void StringTensor<StrClient>::run() {

        sh_mem.attach();

    }

    template <>
    void StringTensor<StrServer>::run() {

        sh_mem.run();

    }

    // class specialization
    template class StringTensor<StrServer>;
    template class StringTensor<StrClient>;


    template <typename ShMemType>
    StringTensor<ShMemType>::~StringTensor() {

        sh_mem.close();

    }



    template <typename ShMemType>
    void StringTensor<ShMemType>::write(const std::vector<std::string>& vec,
                                   int index) {


    }

    template <typename ShMemType>
    void StringTensor<ShMemType>::read(const std::vector<std::string>& vec,
                                   int index) {


    }

    template <typename ShMemType>
    void StringTensor<ShMemType>::close() {

        sh_mem.close();

    }

}
