#pragma once
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include "logger.hpp"


class MCULogger
{
public:
    // Call once from main() or Stmi2wNode constructor.
    // All LOG_* calls across the entire HAL then go to:
    //   ~/logs/StmHAL/<date>/<timestamp>.log
    static void init(const std::string& componentName = "MCU")
    {
        const char* home = std::getenv("HOME");
        if (!home) {
            LOG_ERROR("StmHalLogger", "HOME env not set, logging to console only");
            return;
        }

        auto now  = std::chrono::system_clock::now();
        auto tt   = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&tt);

        std::ostringstream ts, date;
        ts   << std::put_time(&tm, "%Y%m%d_%H%M%S");
        date << std::put_time(&tm, "%d_%b_%Y");

        std::string logDir  = std::string(home) + "/logs/" + componentName + "/" + date.str();
        std::string logFile = logDir + "/" + ts.str() + ".log";

        std::filesystem::create_directories(logDir);

        // configure once — all LOG_* macros across every class share this
        Logger::getInstance().configure(
            Logger::LogLevel::DEBUG,
            logFile,
            false,   // async
            "",      // prefix
            true     // also print to console
        );

        LOG_INFO("MCULogger", "Log file: " + logFile);
    }

    // Prevent instantiation — this is a pure static utility
    MCULogger()  = delete;
    ~MCULogger() = delete;
};
