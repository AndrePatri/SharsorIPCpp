#include <iostream>
#include <SharsorIPCpp/CondVar.hpp>
#include <string>
#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;

Tensor<int> shared_data(1, 1);
Tensor<int> dones_data(1, 1);

bool check_dones(Server<int>& shared_var,
        ConditionVariable& all_done) {
    // Move shared_var.read inside the check_data function
    shared_var.read(dones_data, 0, 0);

    if (dones_data(0, 0) >= 1) {

        shared_var.dataSemAcquire();

        dones_data(0, 0) = 0; // resets counter
        shared_var.write(dones_data, 0, 0);

        shared_var.dataSemRelease();

        return true;

    } else {

        return false;
    }
}

int main() {
    
    std::string name_space = "gthyhgfdghbnvc";

    ConditionVariable cond_var1 = ConditionVariable(true, 
                            "ConVarRead", 
                            name_space,
                            true);

    ConditionVariable cond_var2 = ConditionVariable(true, 
                            "ConVarWrite", 
                            name_space,
                            true);

    Server shared_var = Server<int>(1, 1,
            "VarToBeChecked", 
            name_space,
            true,
            VLevel::V2,
            true, 
            true);
    shared_var.run();

    Server dones_var = Server<int>(1, 1,
            "DonesVar", 
            name_space,
            true,
            VLevel::V2,
            true, 
            false);
    dones_var.run();
    
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    shared_data(0, 0) = 0;

    auto lock2 = cond_var2.lock();

    bool success = true;
    for (int i = 0; i < 100000; ++i) {

        if (success) {

            // auto lock1 = cond_var1.lock();

            // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            shared_data(0, 0) = shared_data(0, 0) + 1;

            shared_var.write(shared_data, 0, 0);
            std::cout << "Wrote " << shared_data << std::endl;

            cond_var1.notify_all();

            success = cond_var2.timedwait_for(lock2, 
                    5000,
                    std::bind(check_dones, std::ref(dones_var), std::ref(cond_var2)));

            if (!success) {
                std::cout << "Timeout reached  " << std::endl;
            }
        }
    }

    cond_var1.close();
    cond_var2.close();

    shared_var.close();

    return 0;
}

