#ifndef RETURNCODES_HPP
#define RETURNCODES_HPP

namespace SharsorIPCpp {

    enum class ReturnCodes: unsigned long long { // 64 bit --> max 64 unique base codes
        MEMOPENED = 1ULL << 0, // opened memory
        MEMCREAT = 1ULL << 1, // created memory
        MEMSET = 1ULL << 2, // set memory
        MEMEXISTS = 1ULL << 3, // memory already exists
        MEMCLEAN = 1ULL << 4, // cleaned memory
        MEMFDCLOSED = 1ULL << 5, // closed file descriptor for memory
        MEMUNLINK = 1ULL << 6, // unlinked memory
        MEMMAPFAIL = 1ULL << 7, // failed to map memory
        INDXOUT = 1ULL << 8, // indexes are out of bound for mem
        NOFIT = 1ULL << 9, // tensor does not fit in memory
        SEMACQTIMEOUT = 1ULL << 10, // semaphore acquisition timeout
        SEMACQ = 1ULL << 11, // semaphore acquired
        SEMACQFAIL = 1ULL << 12, // semaphore acquisition failed
        SEMACQRETRY = 1ULL << 13, // retrying semaphore acquisition
        SEMOPEN = 1ULL << 14, // semaphore opened
        SEMOPENFAIL = 1ULL << 15, // failed to open semaphore
        SEMREL = 1ULL << 16, // reselased semaphore
        SEMRELFAIL = 1ULL << 17, // failed to release semaphore
        SEMCLOSE = 1ULL << 18, // close semaphore
        SEMUNLINK = 1ULL << 19, // unlinked semaphore
        // ... up to 1ULL << 62
        UNKNOWN = 1ULL << 63,

    };

    ReturnCodes operator+(ReturnCodes lhs, ReturnCodes rhs) {

        return static_cast<ReturnCodes>(static_cast<unsigned long long>(lhs) |
                                        static_cast<unsigned long long>(rhs));

    }

    // Example usage:
    // auto combinedCode = ReturnCodes::MEMOPENED + ReturnCodes::MEMEXISTS;
    // if (combinedCode == (ReturnCodes::MEMOPENED + ReturnCodes::MEMEXISTS)) { ... }
    // ReturnCodes::MEMOPENED + ReturnCodes::MEMOPENED will equal ReturnCodes::MEMOPENED

}
#endif // RETURNCODES_HPP

