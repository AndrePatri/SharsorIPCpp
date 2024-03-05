#include <iostream>
#include <SharsorIPCpp/CondVarWrapper.hpp>
#include <string>
#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <csignal>
#include <cstdlib>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using ConditionWrapper = SharsorIPCpp::ConditionWrapper;

std::string name_space = "rthytrtkjdhsxdhn";

int n_writes = 1000000;
int n_expected_listeners = 2;

// Signal handler function
void interruptHandler(int signal) {
    std::cout << "Interrupt signal received (Ctrl+C pressed)." << std::endl;
    std::exit(signal);
}

bool send_signal(Server<bool>::Ptr trigger_ptr,
            Tensor<bool> trigger_data) {

    std::cout << "Sending signal!!" << std::endl;
    
    trigger_data.col(0).setConstant(true); // trigger all listeners
    bool success = trigger_ptr->write(trigger_data, 0, 0);

    return success;
}

bool check_acknowledgment(Server<int>::Ptr ack_ptr, 
            Tensor<int> ack_data) {

    bool success = ack_ptr->read(ack_data, 0, 0);
    
    if (ack_data(0, 0) == 2 && success) {

        std::cout << "Data was read by all listeners!!" << std::endl;

        ack_data(0, 0) = 0; // reset ack counter
        ack_ptr->write(ack_data, 0, 0);

        return true;

    } else {

        return false;
    }

}

int main() {

    std::signal(SIGINT, interruptHandler);

    Server<int>::Ptr ack_ptr = std::make_shared<Server<int>>(1, 1,
                "AcknowledgeData", 
                name_space,
                true,
                VLevel::V2,
                true, 
                false);

    ack_ptr->run();
    Server<bool>::Ptr trigger_ptr = std::make_shared<Server<bool>>(n_expected_listeners, 1,
                "SignalFlag", 
                name_space,
                true,
                VLevel::V2,
                true, 
                false);
    trigger_ptr->run();

    Tensor<int> acknowledge_data(1, 1);
    acknowledge_data(0, 0) = 0; 
    ack_ptr->write(acknowledge_data, 0, 0);

    Tensor<bool> trigger_flags(1, 1);
    trigger_flags.col(0).setConstant(false);
    trigger_ptr->write(trigger_flags, 0, 0);

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

    for (int i = 0; i < n_writes; ++i) {

        condition_write.notify(std::bind(send_signal, 
                                        trigger_ptr, trigger_flags),
                                true);
            
        condition_read.wait(std::bind(check_acknowledgment, 
                                ack_ptr, acknowledge_data), 10000);

                                        
    }

    condition_write.close();

    ack_ptr->close();

    return 0;
}

