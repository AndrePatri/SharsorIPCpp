#include <iostream>
#include <SharsorIPCpp/CondVar.hpp>
#include <string>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;

static Tensor<int> shared_data(1, 1);
Tensor<int> dones_data(1, 1);

// Pass shared_var as a parameter to the check_data function
bool check_data(Client<int>& shared_var,
        ConditionVariable& read_done, 
        Client<int>& shared_dones) {
    // Move shared_var.read inside the check_data function
    shared_var.read(shared_data, 0, 0);

    std::cout << "Var changed. Got: " << shared_data << std::endl;

    shared_dones.dataSemAcquire();
    shared_dones.read(dones_data, 0, 0);
    dones_data(0, 0) = dones_data(0, 0) + 1;

    std::cout << "writing  " << dones_data<< std::endl;
    shared_dones.write(dones_data, 0, 0);
    shared_dones.dataSemRelease();
    
    read_done.notify_one(); // notify if having read

    return shared_data(0, 0) >= 13;
}

int main() {

    static Tensor<int> shared_data(1, 1);

    std::string name_space = "dfsrgthyjukyju";
    
    ConditionVariable cond_var1 = ConditionVariable(false, 
                            "ConVarRead", 
                            name_space,
                            true);

    ConditionVariable cond_var2 = ConditionVariable(false, 
                            "ConVarWrite", 
                            name_space,
                            true);

    Client shared_var = Client<int>("VarToBeChecked", 
            name_space,
            true,
            VLevel::V2,
            true);
    shared_var.attach();

    Client dones_var = Client<int>("DonesVar", 
            name_space,
            true,
            VLevel::V2,
            false);
    dones_var.attach();

    Tensor<int> check(1, 1);
    check(0, 0) = 13;
    
    shared_var.read(shared_data, 0, 0);

    auto lock1 = cond_var1.lock();
    cond_var1.wait_for(lock1, 
        std::bind(check_data, std::ref(shared_var), 
                                std::ref(cond_var2),
                                std::ref(dones_var)));
        
    cond_var1.close();
    cond_var2.close();

    shared_var.close();

    return 0;
}

