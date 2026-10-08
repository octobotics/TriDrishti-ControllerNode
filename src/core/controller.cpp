// #include "robot/robot.hpp"
// #include "managers/i2wManager.hpp"
#include "core/controller.hpp"
#include <ostream>
#include <iostream>

Controller::Controller(/* args */)
{
    LOG_INFO("Controller", "Controller created");   
}

Controller::~Controller()
{
        LOG_INFO("Controller", "Controller Distroyed");   
}
