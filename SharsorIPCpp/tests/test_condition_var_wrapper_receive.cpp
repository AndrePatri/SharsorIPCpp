#include <iostream>
#include <SharsorIPCpp/CondVarWrapper.hpp>
#include <string>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <csignal>
#include <cstdlib>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using ConditionWrapper = SharsorIPCpp::ConditionWrapper;

std::string name_space = "rthytrtkjdhsxdhn";

int n_reads = 1000000;
int counter = 0;
int listener_idx = 0;

// Signal handler function
void interruptHandler(int signal) {
    std::cout << "Interrupt signal received (Ctrl+C pressed)." << std::endl;

    
    std::exit(signal);
}

bool check_trigger(Client<bool>::Ptr trigger_ptr, 
                Tensor<bool> trigger_data,
                int listener_idx) {

    trigger_ptr->read(trigger_data, 0, 0);

    std::cout << "AAAAAAAAAAAAAAAAa" << std::endl;

    if (trigger_data(listener_idx, 0)) {
        
        trigger_data(listener_idx, 0) = false; // reset trigger
        trigger_ptr->write(trigger_data, 0, 0);


        return true;
        
    } else {

        return false;
    }
    
}

bool acknowledge(Client<int>::Ptr ack_ptr,
            Tensor<int> ack_data,
            int listener_idx) {
    
    counter = counter + 1;

    bool read_ok = ack_ptr->read(ack_data, 0, 0);

    if (read_ok) {
        
        ack_data(0, 0) = ack_data(0, 0) + 1;
        ack_ptr->write(ack_data, 0, 0);
    }

    return read_ok;
    
}

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <idx>" << std::endl;
        return 1;
        
    }
    // Get the command line argument for idx
    std::string idxStr = argv[1];

    try {
        // Convert the string to an integer using std::stoi
        listener_idx = std::stoi(idxStr);

        // Use the value of idx in your program
        std::cout << "Will listen at index: " << listener_idx << std::endl;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid argument: " << e.what() << std::endl;
        return 1;
    } catch (const std::out_of_range& e) {
        std::cerr << "Out of range error: " << e.what() << std::endl;
        return 1;
    }

    std::signal(SIGINT, interruptHandler);

    Client<int>::Ptr ack_ptr = std::make_shared<Client<int>>("AcknowledgeData", 
                name_space,
                true,
                VLevel::V2,
                true);
    ack_ptr->attach();

    Client<bool>::Ptr triggers_ptr = std::make_shared<Client<bool>>("SignalFlag", 
                name_space,
                true,
                VLevel::V2,
                true);
    triggers_ptr->attach();

    Tensor<int> acknowledge_data(ack_ptr->getNRows(), 1);
    Tensor<bool> trigger_flags(triggers_ptr->getNRows(), 1);

    ack_ptr->read(acknowledge_data, 0, 0);
    triggers_ptr->read(trigger_flags, 0, 0);

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

    while (true) {
        
        condition_write.wait(std::bind(check_trigger, 
                                        triggers_ptr, trigger_flags, listener_idx), 10000);
            
        // bool success = condition_write.notify(std::bind(acknowledge, ack_ptr, acknowledge_data, listener_idx),
        //                         true);

    }
        

    std::cout << "Contatore "<< counter << std::endl;
    
    condition_write.close();

    ack_ptr->close();

    return 0;
}
