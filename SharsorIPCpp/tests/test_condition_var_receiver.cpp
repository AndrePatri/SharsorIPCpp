#include <iostream>
#include <SharsorIPCpp/CondVar.hpp>
#include <string>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using NamedMutex = ConditionVariable::NamedMutex;
using ScopedLock = ConditionVariable::ScopedLock;

static Tensor<bool> shared_data(1, 1);
Tensor<int> dones_data(1, 1);
std::string name_space = "WEWEW";

int n_reads = 1000000;


bool check_condition() {

    std::cout << "Received signal!!" << std::endl;

    return true;
}

int main() {

    Client shared_var = Client<bool>("VarToBeChecked", 
                name_space,
                true,
                VLevel::V2,
                true);

    shared_var.attach();
    shared_var.read(shared_data, 0, 0);

    ConditionVariable written_condition = ConditionVariable(false, 
                            "ConVarRead", 
                            name_space,
                            true);

    ScopedLock data_lock = written_condition.lock(); // locks data mutex

    if (!check_condition()) {
        
        written_condition.timedwait_for(data_lock, 
            1000, 
            check_condition); // atomically reselases the mutex
            // and acquires it again before exiting

        ConditionVariable::unlock(data_lock); // release data
    }

    written_condition.close();
//     received_condition.close();
    shared_var.close();

    return 0;
}

