#ifndef RETURNCODES_HPP
#define RETURNCODES_HPP

namespace SharsorIPCpp {

    enum class ReturnCode: unsigned long long { // 64 bit --> max 64 unique base codes
        RESET = 0, // clear/reset code
        MEMOPEN = 1ULL << 0, // opened memory
        MEMOPENFAIL = 1ULL << 1, // failed to open memory
        MEMCREAT = 1ULL << 2, // created memory
        MEMCREATFAIL = 1ULL << 3, // failed to create memory
        MEMSET = 1ULL << 4, // memory set
        MEMSETFAIL = 1ULL << 5, // failed to set memory
        MEMEXISTS = 1ULL << 6, // memory already exists
        MEMCLEAN = 1ULL << 7, // cleaned memory
        MEMFDCLOSED = 1ULL << 8, // closed file descriptor for memory
        MEMUNLINK = 1ULL << 9, // unlinked memory
        MEMMAP = 1ULL << 10, // mapped memory
        MEMMAPFAIL = 1ULL << 11, // failed to map memory
        INDXOUT = 1ULL << 12, // indexes are out of bound for mem
        NOFIT = 1ULL << 13, // tensor does not fit in memory
        SEMACQTIMEOUT = 1ULL << 14, // semaphore acquisition timeout
        SEMACQ = 1ULL << 15, // semaphore acquired
        SEMACQFAIL = 1ULL << 16, // semaphore acquisition failed
        SEMACQRETRY = 1ULL << 17, // retrying semaphore acquisition
        SEMOPEN = 1ULL << 18, // semaphore opened
        SEMOPENFAIL = 1ULL << 19, // failed to open semaphore
        SEMREL = 1ULL << 20, // reselased semaphore
        SEMRELFAIL = 1ULL << 21, // failed to release semaphore
        SEMCLOSE = 1ULL << 22, // close semaphore
        SEMUNLINK = 1ULL << 23, // unlinked semaphore
        // ... up to 1ULL << 62
        UNKNOWN = 1ULL << 63,

    };

    // inline to avoid inclusion in multiple translation units
    inline ReturnCode operator+(ReturnCode lhs, ReturnCode rhs) {

        if (rhs == ReturnCode::RESET || lhs == ReturnCode::RESET) {

            return ReturnCode::RESET;

        }

        return static_cast<ReturnCode>(static_cast<unsigned long long>(lhs) |
                                       static_cast<unsigned long long>(rhs));
    }

    inline bool isin(ReturnCode subcode,
              ReturnCode code) {

        return (static_cast<unsigned long long>(code) &
                static_cast<unsigned long long>(subcode)) ==
                    static_cast<unsigned long long>(subcode);
    }

    // Example usage:
    // auto combinedCode = ReturnCode::MEMOPENED + ReturnCode::MEMEXISTS;
    // if (combinedCode == (ReturnCode::MEMOPENED + ReturnCode::MEMEXISTS)) { ... }
    // ReturnCode::MEMOPENED + ReturnCode::MEMOPENED will equal ReturnCode::MEMOPENED
    // code = combinedCode + InfoCodes::RESET; // This will set `code` to InfoCodes::RESET
}
#endif // RETURNCODES_HPP

