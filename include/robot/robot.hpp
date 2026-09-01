#pragma once

#include "managers/i2wManager.hpp"
namespace Robot
{

    class Robot
    {
    public:
        static Robot &instance();

        void start();
        void stop();
        void setupI2w();
        void tick();

    private:
        Robot() = default;
        ~Robot() = default;
        i2wManager m_i2w;

        Robot(const Robot &) = delete;
        Robot &operator=(const Robot &) = delete;
    };

} // namespace Robot