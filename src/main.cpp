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