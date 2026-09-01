#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

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

    std::cout << "Starting Robot..." << std::endl;

    Robot::Robot& robot = Robot::Robot::instance();
    robot.start();
   // namespace
    while (!shutdownRequested.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::cout << "\nStopping Robot..." << std::endl;

    robot.stop();

    return EXIT_SUCCESS;
#include <iostream>
#include <i2w/impl.hpp>
#include <logger.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"


int main()
{   Logger::getInstance().configure(Logger::LogLevel::DEBUG, "robot.log", false);
    Logger &log = Logger::getInstance();

    LOG_DEBUG("Main", "Robot Entry Point... ");

    return 0;
}