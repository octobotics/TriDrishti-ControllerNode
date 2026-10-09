
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <string>

#include <i2w/impl.hpp>
#include "mcuLogger.hpp"
#include "i2wControllerNode.hpp"

namespace
{
    volatile std::sig_atomic_t shutdownRequested = 0;

    void signalHandler(int)
    {
        shutdownRequested = 1;
        
    }
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    MCULogger::init("mcu");

    const auto networkProfileFilePath =
        std::string(CONFIG_DIR) + "/ecal-network-udp.yaml";

    std::cout << "Network Profile File Path "
              << networkProfileFilePath << '\n';

    LOG_INFO("main", "Starting Robot...");

    i2w::Config config;
    config.node_name = "controller_node";
    config.ns = "";

    I2wControllerNode node(config);

    node.Setup();

    while (shutdownRequested == 0)
    {
        node.Tick();

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    LOG_INFO("main", "Stopping Robot...");

    // TODO: Call the node's shutdown/cleanup method here,
    // if its API provides one.

    // node.connection_monitor_running_.store(false);

    LOG_INFO("main", "Robot stopped.");

    return 0;
}