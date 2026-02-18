#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"

#include <booster/robot/channel/channel_factory.hpp>

std::unique_ptr<LowCmd_t>  FSMState::lowcmd  = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard>   FSMState::keyboard = nullptr;

void init_fsm_state()
{
    FSMState::lowcmd   = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Booster Robotics --- \n";
    std::cout << "     Booster K1 Controller \n";

    // Booster DDS init
    booster::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [LT + up] to enter FixStand mode.\n";
    std::cout << "And then press [RT + A] to start controlling the robot.\n";

    while (true)
    {
        sleep(1);
    }

    return 0;
}
