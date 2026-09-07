#pragma once

#include <iostream>
#include <memory>

#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
#include "crawler_i2w_msgs/robot/cmd_vel.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp"
#include "crawler_i2w_services/rasterManualControl.hpp"

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
    i2w::Publisher<crawler_i2w_msgs::cmd_vel> publisher_{};
    i2w::Subscription<crawler_i2w_msgs::JoyMsgs> sub_{};
    crawler_i2w_msgs::cmd_vel cmd_vel_;
    i2w::Client<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse> uiRobotConnectionCheckclient_{};

    float normalize(int16_t value, float max_output);

    bool waiting_for_response_{false};
    bool is_ui_live_{false};
    float speed_factor{1.0f}; // 1 second
    std::chrono::steady_clock::time_point next_call_{};
    std::chrono::steady_clock::time_point response_deadline_{};
    void callUiRobotConnectionCheckService();

    i2w::Client<crawler_i2w_services::RasterLinearActuatorMoveRequest, crawler_i2w_services::RasterLinearActuatorMoveResponse> rasterLinearActuatorMoveClient_{};
    i2w::Client<crawler_i2w_services::RasterProbStopRequest, crawler_i2w_services::RasterProbStopResponse> rasterProbStopClient_{};
    i2w::Client<crawler_i2w_services::RasterProbMoveRequest, crawler_i2w_services::RasterProbMoveResponse> rasterProbMoveClient_{};
    i2w::Client<crawler_i2w_services::RasterProbHomeRequest, crawler_i2w_services::RasterProbHomeResponse> rasterProbHomeClient_{};



void setUiLive(bool live);

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
};
