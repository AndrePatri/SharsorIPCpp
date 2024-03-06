#include <iostream>
#include <SharsorIPCpp/Producer.hpp>
#include <string>
#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>

#include <csignal>
#include <cstdlib>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using Producer = SharsorIPCpp::Producer;

int n_writes = 1000000;
int n_consumers = 2;
bool terminated = false;

// Signal handler function
void interruptHandler(int signal) {
    std::cout << "Interrupt signal received (Ctrl+C pressed)." << std::endl;
    terminated = true;
}

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <namespace>" << std::endl;
        return 1;
        
    }
    
    std::string name_space = argv[1];

    std::signal(SIGINT, interruptHandler);

    Producer producer = Producer("ProducerConsumerTests", 
                            name_space,
                            true,
                            VLevel::V2,
                            false);

    producer.run();
    
    while(!terminated) {

        producer.trigger();

        producer.wait_ack_from(n_consumers);

    }

    producer.close();

    return 0;
}

