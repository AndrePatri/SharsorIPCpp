#include <iostream>
#include <SharsorIPCpp/CondVarWrapper.hpp>
#include <string>
#include <SharsorIPCpp/Server.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using ConditionWrapper = SharsorIPCpp::ConditionWrapper;

static Tensor<bool> shared_data(1, 1);
Tensor<int> dones_data(1, 1);
std::string name_space = "sdfgbgfdswdfv";

int n_reads = 1000000;

bool write_data(Server<bool>& data) {

    std::cout << "Writing data!!" << std::endl;

    shared_data(0, 0) = true;

    return data.write(shared_data, 0, 0);
}

int main() {

    Server shared_var = Server<bool>(1, 1,
                "CrucialData", 
                name_space,
                true,
                VLevel::V2,
                true, 
                false);

    shared_var.run();
    shared_var.read(shared_data, 0, 0);

    ConditionWrapper condition_write = ConditionWrapper(true, 
                            "ConVarWrapperRead", 
                            name_space,
                            true);


    bool success = condition_write.notify(std::bind(write_data, shared_var));

    condition_write.close();

    shared_var.close();

    return 0;
}

