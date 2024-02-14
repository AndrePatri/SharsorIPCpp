#include <iostream>
#include <SharsorIPCpp/CondVar.hpp>
#include <string>

using namespace SharsorIPCpp;

int main() {
    
    ConditionVariable cond_variable_trigger = ConditionVariable(true, 
                            "ConditonVarTestWait", 
                            "Pippo",
                            true);

    ConditionVariable cond_variable_wait = ConditionVariable(true, 
                        "ConditonVarTestTrigger", 
                        "Pippo",
                        true);

    while (true) {

        cond_variable_trigger.notify_all();

        std::cout << "Sender: triggering and waiting" << std::endl;

        cond_variable_wait.wait();

        std::cout << "Sender: Received signal, triggering back" << std::endl;

        // Sleep for a short duration before the next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    cond_variable_wait.close();
    cond_variable_trigger.close();

    return 0;
}

