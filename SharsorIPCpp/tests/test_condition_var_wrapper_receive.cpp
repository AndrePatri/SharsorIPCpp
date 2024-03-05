#include <iostream>
#include <SharsorIPCpp/CondVarWrapper.hpp>
#include <string>
#include <SharsorIPCpp/Client.hpp>
#include <SharsorIPCpp/Journal.hpp>

using namespace SharsorIPCpp;
using LogType = Journal::LogType;
using VLevel = Journal::VLevel;
using ConditionWrapper = SharsorIPCpp::ConditionWrapper;

static Tensor<bool> shared_data(1, 1);
Tensor<int> dones_data(1, 1);
std::string name_space = "sdfgbgfdswdfv";

int n_reads = 1000000;

bool check_data() {

    std::cout << "Checking data!!" << std::endl;

    return true;
}

int main() {

    Client shared_var = Client<bool>("CrucialData", 
                name_space,
                true,
                VLevel::V2,
                true);

    shared_var.attach();
    shared_var.read(shared_data, 0, 0);

    ConditionWrapper condition_write = ConditionWrapper(false, 
                            "ConVarWrapperRead", 
                            name_space,
                            true);

    condition_write.wait(check_data, 1000);

    condition_write.close();

    shared_var.close();

    return 0;
}
