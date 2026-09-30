#include "robot/robot.hpp"
#include "managers/i2wManager.hpp"
#include <ostream>
#include <iostream>

namespace Robot
{

    Robot &Robot::instance()
    {
        static Robot robot;
        return robot;
    }

    void Robot::setupI2w()
    {

        m_i2w.config();
        m_i2w.init();

        m_i2w.setup();
    }

    void Robot::start()
    {
        setupI2w();

        // Initialize hardware
        // Start threads
        // Enter main loop
        LOG_INFO("Robot", "Start Robot");
    }

    void Robot::tick()
    {
        m_i2w.tick();
    }

    void Robot::stop()
    {
        // Stop threads
        // Close devices
        // Release resources

        m_i2w.dispose();
        LOG_INFO("Robot", "Stop Robot");
    }

} // namespace Robot