#include <iostream>
#include <chrono>
#include <thread>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

int main() {
    // Open named condition variable
    boost::interprocess::named_condition condition(
        boost::interprocess::open_or_create, "my_named_conditionu"
    );

    // Open named mutex
    boost::interprocess::named_mutex mutex(
        boost::interprocess::open_or_create, "my_named_mutexau"
    );

    while (true) {
        std::cout << "Receiver: Waiting for signal" << std::endl;
        {
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mutex);
            condition.wait(lock);
        }
        std::cout << "Receiver: Received signal, performing some work" << std::endl;

        // Sleep for a short duration before the next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Clean up (this part is unreachable in an infinite loop)
    return 0;
}

