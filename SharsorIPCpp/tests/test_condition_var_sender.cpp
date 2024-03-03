#include <iostream>
#include <SharsorIPCpp/CondVar.hpp>
#include <string>
#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;

int main() {
    
    std::string name_space = "RERERETTEYY";

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

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    Tensor<int> shared_data(1, 1);
    shared_data(0, 0) = 0;

    auto lock2 = cond_var2.lock();

    for (int i = 0; i < 13; ++i) {

        // auto lock1 = cond_var1.lock();

        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        shared_data(0, 0) = shared_data(0, 0) + 1;

        shared_var.write(shared_data, 0, 0);
        std::cout << "Wrote " << shared_data << std::endl;

        cond_var1.notify_all();

        cond_var2.wait(lock2);

    }

    cond_var1.close();
    cond_var2.close();

    shared_var.close();

    return 0;
}

