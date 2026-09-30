#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <iostream>
#include <i2w/impl.hpp>
// #include <logger.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "mcuLogger.hpp"


#include "robot/robot.hpp"

namespace
{
    std::atomic<bool> shutdownRequested{false};

    void signalHandler(int)
    {
        shutdownRequested.store(true, std::memory_order_relaxed);
    }
} // namespace

int main()
{
    std::signal(SIGINT, signalHandler);

        mcu::MCULogger::init("mcu");


    auto networkProfileFilePath = std::string(CONFIG_DIR) + "/ecal-network-udp.yaml";
    
    std::cout << "Network Profile File Path "<<networkProfileFilePath << std::endl;

    std::cout << "Starting Robot..." << std::endl;

    Robot::Robot &robot = Robot::Robot::instance();
    robot.start();
    // namespace
    while (!shutdownRequested.load(std::memory_order_relaxed))
    {
        robot.tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::cout << "\nStopping Robot..." << std::endl;

    robot.stop();


    return 0;
}