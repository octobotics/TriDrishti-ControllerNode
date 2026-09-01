#pragma once

#include <iostream>
#include <memory>

#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
#include "crawler_i2w_msgs/robot/cmd_vel.hpp"


class i2wNode;

class i2wManager
{
public:
    i2wManager();
    ~i2wManager();

    void config();
    void init();
    void setup();
    void tick();
    void dispose();

private:
    i2w::Config m_robotMcuConfig;
    std::unique_ptr<i2wNode> m_robotMcuNode;
};

class i2wNode final : public i2w::SystemBase
{
public:
    explicit i2wNode(i2w::Config config)
        : SystemBase(std::move(config))
    {
        std::cout << "i2wNode Constructed\n";
    }

    ~i2wNode()
    {
        std::cout << "i2wNode Destroyed\n";
    }

    // it should be onSetUp
    i2w::LifecycleResult OnSetup() noexcept;
    // it should be onTick
    i2w::LifecycleResult OnTick() noexcept;
};