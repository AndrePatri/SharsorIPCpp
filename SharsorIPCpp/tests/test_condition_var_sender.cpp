#include <iostream>
#include <chrono>
#include <thread>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

int main() {
    // Create or open named condition variable
    boost::interprocess::named_condition condition(
        boost::interprocess::open_or_create, "my_named_conditionu"
    );

    // Create or open named mutex
    boost::interprocess::named_mutex mutex(
        boost::interprocess::open_or_create, "my_named_mutexau"
    );

    while (true) {
        std::cout << "Sender: Performing some work" << std::endl;
        // Do some work...

        // Send signal to the other process
        std::cout << "Sender: Sending signal to the receiver process" << std::endl;
        {
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mutex);
            condition.notify_one();
        }

        // Sleep for a short duration before the next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Clean up (this part is unreachable in an infinite loop)
    condition.remove("my_named_condition");
    mutex.remove("my_named_mutex");

    return 0;
}

