#include "managers/i2wManager.hpp"

i2wManager::i2wManager()
{
    std::cout << "Initializing i2w Manager\n";
}

i2wManager::~i2wManager()
{
    dispose();
    std::cout << "i2w Manager Closed\n";
}

void i2wManager::config()
{
    std::cout << "Configuring MCU Node\n";

    m_robotMcuConfig.node_name = "robotMcu";
    m_robotMcuConfig.ns = "robot";
    m_robotMcuConfig.transport.network_profile_file =
        "src/TriDrishti-ControllerNode/config/ecal-network-udp.yaml";
}

void i2wManager::init()
{
    std::cout << "Initializing MCU Node\n";

    m_robotMcuNode = std::make_unique<i2wNode>(m_robotMcuConfig);
}

void i2wManager::setup()
{
    if (!m_robotMcuNode)
    {
        std::cerr << "ERROR: Node not initialized.\n";
        return;
    }

    m_robotMcuNode->Setup();
}

void i2wManager::tick()
{
    if (!m_robotMcuNode)
        return;

    m_robotMcuNode->Tick();
}

void i2wManager::dispose()
{
    if (m_robotMcuNode)
    {
        std::cout << "Disposing MCU Node\n";
        m_robotMcuNode.reset();
    }
}