#ifndef SHAREDMEMCONFIG_HPP
#define SHAREDMEMCONFIG_HPP

#include <string>
#include <SharsorIPCpp/MemDefs.hpp>

namespace SharsorIPCpp{

    class SharedMemConfig {

    public:

        SharedMemConfig(const std::string& name,
                        const std::string& name_space = "")
            : _name(name),
              _namespace(name_space) {

            mem_path = "/" + _namespace + _name;

            mem_path_nrows = "/" + _namespace + _name + "_" + MemDef::sharedTensorNRowsName();

            mem_path_ncols = "/" + _namespace + _name + "_" + MemDef::sharedTensorNColsName();

            mem_path_dtype = "/" + _namespace + _name + "_" + MemDef::sharedTensorDTypeName();

            mem_path_clients_counter = "/" + _namespace + _name + "_" + MemDef::ClientsCountName();

            mem_path_isrunning = "/" + _namespace + _name + "_" + MemDef::isSrvrRunningName();

            mem_path_mem_layout = "/" + _namespace + _name + "_" + MemDef::memLayoutName();

            mem_path_server_sem = "/" + _namespace + _name + "_" + MemDef::SrvrSemName();

            mem_path_data_sem = "/" + _namespace + _name + "_" + MemDef::DataSemName();

        }

        // shared data path
        std::string mem_path;

        // auxiliary data
        std::string mem_path_nrows;
        std::string mem_path_ncols;
        std::string mem_path_dtype;
        std::string mem_path_clients_counter;
        std::string mem_path_isrunning;
        std::string mem_path_mem_layout;

        // semaphores
        std::string mem_path_server_sem;
        std::string mem_path_data_sem;

    private:

        std::string _name;
        std::string _namespace;

    };

}
#endif // SHAREDMEMCONFIG_HPP
