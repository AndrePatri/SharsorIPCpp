#ifndef MEMDEFS_H
#define MEMDEFS_H

using namespace SharsorIPCpp{

    class MemDef {

        public:

            static std::string sharedSrvrNRowsName() {

                return _sharedSrvrNRowsName;

            }

            static std::string sharedSrvrNColsName() {

                return _sharedSrvrNColsName;

            }

            static std::string sharedClientsCountName() {

                return _sharedClientsCountName;

            }

            static std::string sharedSemSrvrName() {

                return _sharedSemSrvrName;

            }

            static std::string sharedSemClientsCountName() {

                return _sharedSemClientsCountName;

            }

        private:

            std::string _sharedSrvrNRowsName = "sharedSrvrNRows";

            std::string _sharedSrvrNColsName = "sharedSrvrNCols";

            std::string _sharedClientsCountName = "sharedClientsCount";

            std::string _sharedSemSrvrName = "sharedSemSrvr";

            std::string _sharedSemClientsCountName = "sharedSemClientsCount";
        };

}

#endif // MEMDEFS_H
