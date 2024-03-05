#include <iostream>
#include <SharsorIPCpp/CondVar.hpp>
#include <string>
#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>
#include <chrono>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using NamedMutex = ConditionVariable::NamedMutex;
using ScopedLock = ConditionVariable::ScopedLock;

Tensor<bool> shared_data(1, 1);
Tensor<int> dones_data(1, 1);
std::string name_space = "WEWEW";

Server shared_var = Server<bool>(1, 1,
            "VarToBeChecked", 
            name_space,
            true,
            VLevel::V2,
            true, 
            false);

int main() {
    
    shared_var.run();
    shared_data(0, 0) = false;
    shared_var.write(shared_data, 0, 0); // writing initialization
    
    ConditionVariable written_condition = ConditionVariable(true, 
                            "ConVarRead", 
                            name_space,
                            true);

    int n_writes = 1000000;

    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < n_writes; ++i) {

        ScopedLock data_lock = written_condition.lock(); // locks data mutex

        // modify shared variable
        shared_data(0, 0) = true;
        shared_var.write(shared_data, 0, 0);
        
        ConditionVariable::unlock(data_lock); // release data

        written_condition.notify_all(); // notify listeners on condition variable
        
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);

    std::cout << "Total write-notify-wait_for for n. it:" << n_writes << " , seconds: "<< duration.count() << std::endl;
    std::cout << "Avrg for write-notify-wait_for " << duration.count()/n_writes << std::endl;

    written_condition.close();
//     received_condition.close();
    shared_var.close();

    return 0;
}

