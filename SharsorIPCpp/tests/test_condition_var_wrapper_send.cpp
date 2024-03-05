#include <iostream>
#include <SharsorIPCpp/CondVarWrapper.hpp>
#include <string>
#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using ConditionWrapper = SharsorIPCpp::ConditionWrapper;

Tensor<int> acknowledge_data(1, 1);
Tensor<bool> signal_flag(1, 1);

std::string name_space = "aesrthsadfghfdgh";

int n_writes = 1000000;
int n_expected_listeners = 2;

bool send_signal(Server<bool>::Ptr data_ptr) {

    std::cout << "Sending signal!!" << std::endl;
    
    signal_flag(0, 0) = true; // reset data
    bool success = data_ptr->write(signal_flag, 0, 0);

    return success;
}

bool check_acknowledgment(Server<int>::Ptr data_ptr, 
            Server<bool>::Ptr flag_ptr) {

    bool success = data_ptr->read(acknowledge_data, 0, 0);
    
    std::cout << "Received ack signal!!" << std::endl;
    std::cout << acknowledge_data(0, 0) << std::endl;

    if (acknowledge_data(0, 0) == 2 && success) {

        std::cout << "Data was read by all listeners!!" << std::endl;

        signal_flag(0, 0) = false; // ready_for next_trigger
        flag_ptr->write(signal_flag, 0, 0);

        return true;

    } else {

        return false;
    }

}

int main() {

    Server<int>::Ptr ack_ptr = std::make_shared<Server<int>>(1, 1,
                "AcknowledgeData", 
                name_space,
                true,
                VLevel::V2,
                true, 
                false);

    ack_ptr->run();
    Server<bool>::Ptr flag_ptr = std::make_shared<Server<bool>>(1, 1,
                "SignalFlag", 
                name_space,
                true,
                VLevel::V2,
                true, 
                false);
    signal_flag(0, 0) = false;
    flag_ptr->run();
    flag_ptr->write(signal_flag, 0, 0);

    // Convert seconds to milliseconds
    std::chrono::milliseconds duration(100);

    // Sleep for the specified duration
    std::this_thread::sleep_for(duration);

    ConditionWrapper condition_write = ConditionWrapper(true, 
                            "ConVarWrapperWrite", 
                            name_space,
                            true,
                            VLevel::V1);
    
    ConditionWrapper condition_read = ConditionWrapper(true, 
                            "ConVarWrapperRead", 
                            name_space,
                            true,
                            VLevel::V1);

    for (int i = 0; i < 1; ++i) {

        bool signal_ok = condition_write.notify(std::bind(send_signal, flag_ptr),
                                        true);

        bool ack_ok = condition_read.wait(std::bind(check_acknowledgment, 
                                    ack_ptr, flag_ptr), 
                                    10);
                                        
    }

    condition_write.close();

    ack_ptr->close();

    return 0;
}

