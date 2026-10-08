#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <iostream>
#include <i2w/impl.hpp>

#include "mcuLogger.hpp"
#include "i2wControllerNode.hpp"


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

    MCULogger::init("mcu");

    auto networkProfileFilePath = std::string(CONFIG_DIR) + "/ecal-network-udp.yaml";

    std::cout << "Network Profile File Path " << networkProfileFilePath << std::endl;

    LOG_INFO("main", "Starting Robot...");

    i2w::Config i2w_mcu_config;
    i2w_mcu_config.node_name = "controller_node";
    i2w_mcu_config.ns = "";

    I2wControllerNode i2wControllerNode(i2w_mcu_config);
    

    i2wControllerNode.Setup();

    while (!shutdownRequested.load(std::memory_order_relaxed))
    {
        i2wControllerNode.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }


    i2wControllerNode.Dispose();

        LOG_INFO("main", "Stopping Robot...");

    return 0;
}