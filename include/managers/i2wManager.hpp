#pragma once

#include <iostream>
#include <memory>

#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
#include "crawler_i2w_msgs/robot/cmd_vel.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp"
#include "crawler_i2w_services/rasterManualControl.hpp"
#include "crawler_i2w_services/moveRobot.hpp"
#include "crawler_i2w_services/controlModeSwitching.hpp"
#include "logger.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <thread>
#include <atomic>
#include <atomic>
#include <csignal>
#include <iostream>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <csignal>

#include "mcuSetting.hpp"

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
    enum class ControlModeType : std::uint8_t
    {
        ManualJoy = crawler_i2w_services::ControlMode::kControlModeManualJoy,
        AutoMission = crawler_i2w_services::ControlMode::kControlModeAutoMission,
    };
    explicit i2wNode(i2w::Config config)
        : SystemBase(std::move(config))
    {
       LOG_INFO("i2wNode","Constructor");
    }

    ~i2wNode()
    {
       LOG_INFO("i2wNode","Disconstructor");
    }

    i2w::LifecycleResult OnSetup() noexcept;
    i2w::LifecycleResult OnTick() noexcept;

    // publisher

    i2w::Publisher<crawler_i2w_msgs::cmd_vel> cmd_velPublisher_{};
    i2w::Publisher<crawler_i2w_msgs::cmd_vel_ui> cmd_vel_uiPublisher_;

    // subscriber

    i2w::Subscription<crawler_i2w_msgs::JoyMsgs> sub_{};
    i2w::Subscription<crawler_i2w_msgs::cmd_vel> cmd_vel_correction_sub_{};
    i2w::Subscription<crawler_i2w_msgs::cmd_vel> cmd_vel_mission_sub_{};


    // service server
    i2w::Server<crawler_i2w_services::MoveRobotRequest, crawler_i2w_services::MoveRobotResponse> move_robot_service_;
    i2w::Server<crawler_i2w_services::ControlModeSwitchingRequest, crawler_i2w_services::ControlModeSwitchingResponse> control_mode_switching_service_;
    i2w::Server<crawler_i2w_services::ControlModeStatusRequest,crawler_i2w_services::ControlModeStatusResponse> control_mode_status_service_;

    // service client
    i2w::Client<crawler_i2w_services::RasterLinearActuatorMoveRequest, crawler_i2w_services::RasterLinearActuatorMoveResponse> rasterLinearActuatorMoveClient_{};
    i2w::Client<crawler_i2w_services::RasterProbStopRequest, crawler_i2w_services::RasterProbStopResponse> rasterProbStopClient_{};
    i2w::Client<crawler_i2w_services::RasterProbMoveRequest, crawler_i2w_services::RasterProbMoveResponse> rasterProbMoveClient_{};
    i2w::Client<crawler_i2w_services::RasterProbHomeRequest, crawler_i2w_services::RasterProbHomeResponse> rasterProbHomeClient_{};
    i2w::Client<crawler_i2w_services::MoveRobotRequest, crawler_i2w_services::MoveRobotResponse> move_robot_client_;
    i2w::Client<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse> uiRobotConnectionCheckclient_{};

    crawler_i2w_msgs::cmd_vel cmd_vel_;
    crawler_i2w_msgs::cmd_vel current_cmd_vel_;
    crawler_i2w_msgs::cmd_vel current_cmd_vel_correction{};
    crawler_i2w_msgs::cmd_vel current_cmd_vel_misssion{};

    bool waiting_for_response_{false};
    bool is_ui_live_{false};
    float speed_factor{1.0f}; // 1 second
    ControlModeType current_mode = ControlModeType::ManualJoy;
    
    std::chrono::steady_clock::time_point next_call_{};
    std::chrono::steady_clock::time_point response_deadline_{};

    void setUiLive(bool live);
    void publishCmd_Vel();
    void callUiRobotConnectionCheckService();
    void configerLogger();
    void publishCmd_Vel_Ui(float maxLinear, float maxAngular, float linear, float angular, bool setMaxValue);
    float normalize(int16_t value, float max_output);

    // i2w service setup helper function
    bool isConnected();

    template <typename RequestType, typename ResponseType, typename ClientType>
    void setupClient(
        i2w::RuntimeHandle &runtime,
        const std::string &serviceName,
        ClientType &destination,
        std::function<void()> onFailure) noexcept
    {

        i2w::ServiceOptions opts;
        opts.plane = i2w::EndpointPlane::Local;
        opts.max_outstanding_calls = 10;
        opts.call_timeout_ms = 2000;

        auto client = runtime.create_client<RequestType, ResponseType>(serviceName, opts);
        if (!client)
        {

            if (onFailure)
            {
                onFailure();
            }

            return;
        }

        destination = std::move(client.value());
    }

    template <typename RequestT, typename ResponseT, typename ServiceMemberT, typename CallbackT>
    bool advertiseService(
        const std::string &topic,
        ServiceMemberT &serviceMember,
        CallbackT &&callback,
        uint32_t call_timeout_ms = 1000,
        uint32_t max_requests_per_spin = 100,
        uint32_t max_responses_per_spin = 100,
        uint32_t server_queue_depth = 64)
    {
        i2w::ServiceOptions options;
        options.plane = i2w::EndpointPlane::Local;
        options.reliability = i2w::Reliability::Reliable;
        options.call_timeout_ms = call_timeout_ms;
        options.max_requests_per_spin = max_requests_per_spin;
        options.max_responses_per_spin = max_responses_per_spin;
        options.server_queue_depth = server_queue_depth;

        auto service = runtime().advertise_service<RequestT, ResponseT>(
            topic, std::forward<CallbackT>(callback), options);

        if (!service)
        {
            // Log("Failed to advertise service: " + topic);
            return false;
        }

        serviceMember = std::move(service.value());
        return true;
    }
};
