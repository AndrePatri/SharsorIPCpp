#include <iostream>
#include <SharsorIPCpp/CondVar.hpp>
#include <string>

using namespace SharsorIPCpp;

int main() {

    bool is_server = true;
    
    ConditionVariable cond_variable_trigger = ConditionVariable(true, 
                            "ConditonVarTestWait", 
                            "Pippo",
                            true);

    ConditionVariable cond_variable_wait = ConditionVariable(true, 
                        "ConditonVarTestTrigger", 
                        "Pippo",
                        true);

    while (true) {

        cond_variable_trigger.wait();

        std::cout << "Receiver: Received signal, triggering back" << std::endl;

        cond_variable_wait.notify_all();

        // Sleep for a short duration before the next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    cond_variable_wait.close();
    cond_variable_trigger.close();
    // Clean up (this part is unreachable in an infinite loop)
    return 0;
}

