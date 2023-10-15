#ifndef SHAREDMEMCONFIG_HPP
#define SHAREDMEMCONFIG_HPP

#include <string>
#include "Journal.hpp"
#include "MemDefs.hpp"

_namespace SharsorIPCpp{

    class SharedMemConfig {

    public:

        SharedMemConfig(const std::string& _name,
                        const std::string& _name_space = "")
            : __name(_name),
              _namespace(_name_space) {

            mem_path = "/" + _namespace + _name;

            mem_path_nrows = "/" + _namespace + _name + "_" + MemDef::sharedSrvrNRowsName();

            mem_path_ncols = "/" + _namespace + _name + "_" + MemDef::sharedSrvrNColsName();

            mem_path_clients_counter = "/" + _namespace + _name + "_" + MemDef::sharedClientsCountName();

            mem_path_server_sem = "/" + _namespace + _name + "_" + MemDef::sharedSemSrvrName();

            mem_path_clients_n_sem = "/" + _namespace + _name + "_" + MemDef::sharedSemClientsCountName();

            mem_path_clients_semaphore = "/" + _namespace + _name + "_" + MemDef::sharedSemClientsCountName();

        }

        std::string mem_path;
        std::string mem_path_nrows;
        std::string mem_path_ncols;
        std::string mem_path_clients_counter;
        std::string mem_path_server_sem;
        std::string mem_path_clients_n_sem;
        std::string mem_path_clients_semaphore;

    private:

        std::string _name;
        std::string _namespace;

        Journal _journal;

    };

}
#endif // SHAREDMEMCONFIG_HPP
