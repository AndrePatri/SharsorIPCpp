#ifndef MEMDEFS_H
#define MEMDEFS_H

namespace SharsorIPCpp{

    class MemDef {

        public:

            static std::string sharedSrvrNRowsName() {

                return std::string("sharedSrvrNRows");

            }

            static std::string sharedSrvrNColsName() {

                return std::string("sharedSrvrNCols");

            }

            static std::string sharedClientsCountName() {

                return std::string("sharedClientsCount");

            }

            static std::string sharedSemSrvrName() {

                return std::string("sharedSemSrvr");

            }

            static std::string sharedSemClientsCountName() {

                return std::string("sharedSemClientsCount");

            }

        };

}

#endif // MEMDEFS_H
