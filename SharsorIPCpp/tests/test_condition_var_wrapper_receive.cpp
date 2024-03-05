#include <iostream>
#include <SharsorIPCpp/CondVarWrapper.hpp>
#include <string>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using ConditionWrapper = SharsorIPCpp::ConditionWrapper;

Tensor<int> acknowledge_data(1, 1);
Tensor<bool> signal_flag(1, 1);

std::string name_space = "aesrthsadfghfdgh";

int n_reads = 1000000;

bool check_signal(Client<bool>::Ptr data_ptr) {

    data_ptr->read(signal_flag, 0, 0);

    if (signal_flag(0, 0)) {

        std::cout << "Received signal!!" << std::endl;

        return true;
        
    } else {

        return false;
    }
    
}

bool acknowledge(Client<int>::Ptr data_ptr) {

    std::cout << "Acknowledging reception!!" << std::endl;

    bool read_ok = data_ptr->read(acknowledge_data, 0, 0);

    if (read_ok) {
        
        acknowledge_data(0, 0) = acknowledge_data(0, 0) + 1;

        data_ptr->write(acknowledge_data, 0, 0);
    }

    return read_ok;
    
}

int main() {

    Client<int>::Ptr ack_ptr = std::make_shared<Client<int>>("AcknowledgeData", 
                name_space,
                true,
                VLevel::V2,
                true);
    ack_ptr->attach();
    ack_ptr->read(acknowledge_data, 0, 0);

    Client<bool>::Ptr flag_ptr = std::make_shared<Client<bool>>("SignalFlag", 
                name_space,
                true,
                VLevel::V2,
                true);
    flag_ptr->attach();
    flag_ptr->read(signal_flag, 0, 0);

    ConditionWrapper condition_write = ConditionWrapper(false, 
                            "ConVarWrapperWrite", 
                            name_space,
                            true,
                            VLevel::V1);

    ConditionWrapper condition_read = ConditionWrapper(false, 
                            "ConVarWrapperRead", 
                            name_space,
                            true,
                            VLevel::V1);

    for (int i = 0; i < 1; ++i) {

        bool signal_received = condition_write.wait(std::bind(check_signal, flag_ptr), 
                                    10);

        if (signal_received) {

            bool success = condition_write.notify(std::bind(acknowledge, ack_ptr),
                                        true);
        }
        
    }
    
    condition_write.close();

    ack_ptr->close();

    return 0;
}
