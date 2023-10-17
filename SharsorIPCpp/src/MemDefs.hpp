#ifndef MEMDEFS_H
#define MEMDEFS_H

namespace SharsorIPCpp{

    class MemDef {

        public:

            static std::string sharedTensorNRowsName() {

                return std::string("sharedTensorNRows");

            }

            static std::string sharedTensorNColsName() {

                return std::string("sharedTensorNCols");

            }

            static std::string sharedTensorDTypeName() {

                return std::string("sharedTensorDType");

            }

            static std::string ClientsCountName() {

                return std::string("clientsCount");

            }

            static std::string SrvrSemName() {

                return std::string("srvrSem");

            }

            static std::string DataSemName() {

                return std::string("dataSem");

            }

        };

}

#endif // MEMDEFS_H
