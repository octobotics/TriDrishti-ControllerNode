#include "i2wControllerNode.hpp"
#include <iostream>
#include "mcuLogger.hpp"

I2wControllerNode::I2wControllerNode(i2w::Config config) : i2w::SystemBase(std::move(config))
{
    LOG_INFO("I2wControllerNode", "I2wControllerNode created");
}

I2wControllerNode::~I2wControllerNode()
{
    LOG_INFO("I2wControllerNode", "I2wControllerNode Distroyed");
}

i2w::LifecycleResult I2wControllerNode::OnSetup() noexcept
{
    LOG_INFO("I2wControllerNode", "OnSetup");

    i2w::SubscriptionOptions opts;
    opts.plane = i2w::EndpointPlane::Local;
    opts.reliability = i2w::Reliability::BestEffort;
    opts.queue_depth = 32;
    opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto subscription = runtime().subscribe<crawler_i2w_msgs::JoyMsgs>(
        "/joy",
        [this](const i2w::Sample<crawler_i2w_msgs::JoyMsgs> &sample)
        {
            if (is_ui_live_)
            {
                if (sample.value.button0)
                {
                    rasterProbHomeClient_.call({0}, static_cast<std::int64_t>(runtime().clock().now().ns), [](const i2w::Sample<crawler_i2w_services::RasterProbHomeResponse> &response)
                                               { std::cout << "Raster Prob Home Service Response: " << (response.value.status ? "Success" : "Failure") << std::endl; });
                }
                if (sample.value.button3)
                {
                    rasterProbStopClient_.call({0}, static_cast<std::int64_t>(runtime().clock().now().ns), [](const i2w::Sample<crawler_i2w_services::RasterProbStopResponse> &response)
                                               { std::cout << "Raster Prob Stop Service Response: " << (response.value.status ? "Success" : "Failure") << std::endl; });
                }
                if (sample.value.button1)
                {
                    rasterProbMoveClient_.call({-1, true}, static_cast<std::int64_t>(runtime().clock().now().ns), [](const i2w::Sample<crawler_i2w_services::RasterProbMoveResponse> &response)
                                               { std::cout << "Raster Prob Move Right Service Response: " << (response.value.status ? "Success" : "Failure") << std::endl; });
                }
                if (sample.value.button4)
                {
                    rasterProbMoveClient_.call({1, false}, static_cast<std::int64_t>(runtime().clock().now().ns), [](const i2w::Sample<crawler_i2w_services::RasterProbMoveResponse> &response)
                                               { std::cout << "Raster Prob Move Left Service Response: " << (response.value.status ? "Success" : "Failure") << std::endl; });
                }

                if (sample.value.button4)
                {
                    speed_factor++;
                    if (speed_factor > 10)
                    {
                        speed_factor = 10;
                    }
                    publishCmd_Vel_Ui(10, 2.5 * speed_factor, 0, 0, 1);

                    std::cout << "Speed Factor: " << speed_factor << std::endl;
                }
                if (sample.value.button0)
                {
                    speed_factor--;
                    if (speed_factor < 1)
                    {
                        speed_factor = 1;
                    }

                    publishCmd_Vel_Ui(10, 2.5 * speed_factor, 0, 0, 1);

                    std::cout << "Speed Factor: " << speed_factor << std::endl;
                }

                current_cmd_vel_.linearVelocity = -normalize(sample.value.axis2, 10);
                current_cmd_vel_.angularVelocity = -normalize(sample.value.axis0, 2.5 * speed_factor);
                current_cmd_vel_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);

                if (current_cmd_vel_.linearVelocity > -1.0f &&
                    current_cmd_vel_.linearVelocity < 1.0f)
                {
                    current_cmd_vel_.linearVelocity = 0.0f;
                }

                if (current_cmd_vel_.angularVelocity > -1.0f &&
                    current_cmd_vel_.angularVelocity < 1.0f)
                {
                    current_cmd_vel_.angularVelocity = 0.0f;
                }

                // cmd val correction value merge

                // cmd_vel_.linearVelocity += current_cmd_vel_correction.linearVelocity;
                // cmd_vel_.angularVelocity += current_cmd_vel_correction.angularVelocity;

                // (void)publisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));
                // std::cout << "Published cmd_vel: linearVelocity -> " << cmd_vel_.linearVelocity << " angularVelocity -> " << cmd_vel_.angularVelocity << std::endl;
            }
        },
        opts);
    sub_ = std::move(subscription.value());

    i2w::SubscriptionOptions optsCmdVelCorrection;
    optsCmdVelCorrection.plane = i2w::EndpointPlane::Local;
    optsCmdVelCorrection.reliability = i2w::Reliability::BestEffort;
    optsCmdVelCorrection.queue_depth = 32;
    optsCmdVelCorrection.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto cmd_vel_correction_sub = runtime().subscribe<crawler_i2w_msgs::cmd_vel>(
        "/cmd_vel_correction",
        [this](const i2w::Sample<crawler_i2w_msgs::cmd_vel> &sample)
        {
            current_cmd_vel_correction = sample.value;
        },
        optsCmdVelCorrection);
    cmd_vel_correction_sub_ = std::move(cmd_vel_correction_sub.value());

    // sub edge status

    auto edge_status_sub = runtime().subscribe<crawler_i2w_msgs::edge_status>(
        "/edge_status",
        [this](const i2w::Sample<crawler_i2w_msgs::edge_status> &sample)
        {
            current_edge_status = sample.value;
        },
        optsCmdVelCorrection);
    edge_status_sub_ = std::move(edge_status_sub.value());

    // mission cmd_vel

    auto cmd_vel_mission_sub = runtime().subscribe<crawler_i2w_msgs::cmd_vel>(
        "/mission/cmd_vel",
        [this](const i2w::Sample<crawler_i2w_msgs::cmd_vel> &sample)
        {
            current_cmd_vel_misssion = sample.value;
        },
        optsCmdVelCorrection);
    cmd_vel_mission_sub_ = std::move(cmd_vel_mission_sub.value());

    i2w::PublisherOptions cmdVelPubOpt;
    cmdVelPubOpt.plane = i2w::EndpointPlane::Local;

    auto cmd_velPublisher = runtime().advertise<crawler_i2w_msgs::cmd_vel>("/cmd_vel", cmdVelPubOpt);

    cmd_velPublisher_ = std::move(cmd_velPublisher.value());

    bool ok = advertiseService<crawler_i2w_services::MoveRobotRequest,
                               crawler_i2w_services::MoveRobotResponse>(
        "/move_robot_service",
        move_robot_service_,
        [this](const crawler_i2w_services::MoveRobotRequest &request, const i2w::Header &header,
               crawler_i2w_services::MoveRobotResponse &response)
        {
            if (is_ui_live_)
            {
                move_robot_client_.call({request.distance, request.speed}, static_cast<std::int64_t>(runtime().clock().now().ns), [](const i2w::Sample<crawler_i2w_services::MoveRobotResponse> &response)
                                        { std::cout << "Raster Prob Move Right Service Response: " << (response.value.result ? "Success" : "Failure") << std::endl; });
                response.result = true;
            }
            else
            {
                std::cout << "Ui is not avaliable, Connent to Ui First" << std::endl;
            }
        });
    if (!ok)
        return i2w::Fail();

    ok = advertiseService<crawler_i2w_services::ControlModeSwitchingRequest,
                          crawler_i2w_services::ControlModeSwitchingResponse>(
        "/controller/control_mode",
        control_mode_switching_service_,
        [this](const crawler_i2w_services::ControlModeSwitchingRequest &request, const i2w::Header &header,
               crawler_i2w_services::ControlModeSwitchingResponse &response)
        {
            LOG_INFO("ControlModeSwitching Service", "Control Mode Switching Request");
            response = {};
            response.success = 1;
            response.active_mode = static_cast<std::uint8_t>(current_mode);

            if (request.requested_mode == crawler_i2w_services::ControlMode::kControlModeManualJoy && current_mode != ControlModeType::ManualJoy)
            {
                current_mode = ControlModeType::ManualJoy;
                response.active_mode = crawler_i2w_services::ControlMode::kControlModeManualJoy;
                response.success = 0;

                LOG_INFO("ControlModeSwitching Service", "Set Control Mode to Manual Joy");
            }
            else if (request.requested_mode == crawler_i2w_services::ControlMode::kControlModeAutoMission && current_mode != ControlModeType::AutoMission)
            {
                current_mode = ControlModeType::AutoMission;
                response.active_mode = crawler_i2w_services::ControlMode::kControlModeAutoMission;
                response.success = 0;

                LOG_INFO("ControlModeSwitching Service", "Set Control Mode to Auto Mission");
            }
            else
            {
                // Keep current_mode and active_mode unchanged.
                response.success = 1U;

                LOG_ERROR("ControlModeSwitching Service", "Invalid requested control mode");
            }

            response.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();
        });
    if (!ok)
        return i2w::Fail();

    ok = advertiseService<
        crawler_i2w_services::ControlModeStatusRequest,
        crawler_i2w_services::ControlModeStatusResponse>(
        "/controller/control_mode_status",
        control_mode_status_service_,
        [this](
            const crawler_i2w_services::ControlModeStatusRequest &,
            const i2w::Header &,
            crawler_i2w_services::ControlModeStatusResponse &response)
        {
            response = {};
            response.success = 0U; // Existing inverted convention: success.
            response.active_mode = static_cast<std::uint8_t>(current_mode);

            std::snprintf(
                response.message.data(), response.message.size(),
                "control mode query successful");

            response.timestamp_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
        });

    if (!ok)
        return i2w::Fail();

    setupClient<crawler_i2w_services::MoveRobotRequest, crawler_i2w_services::MoveRobotResponse>(
        runtime(),
        "/mcu/move_robot_service",
        move_robot_client_,
        []
        {
            std::cerr << "Failed to create client for /ui_robot_connection_check service\n";
        });

    // Setup a client for the service
    setupClient<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse>(
        runtime(),
        "/ui_robot_connection_check",
        uiRobotConnectionCheckclient_,
        []
        {
            std::cerr << "Failed to create client for /ui_robot_connection_check service\n";
        });

    // Raster Service Client Setup

    setupClient<crawler_i2w_services::RasterProbHomeRequest, crawler_i2w_services::RasterProbHomeResponse>(
        runtime(),
        "/raster_prob_home_service",
        rasterProbHomeClient_,
        []
        {
            std::cerr << "Failed to create client for /raster_prob_home_service service\n";
        });

    setupClient<crawler_i2w_services::RasterProbMoveRequest, crawler_i2w_services::RasterProbMoveResponse>(
        runtime(),
        "/raster_prob_move_service",
        rasterProbMoveClient_,
        []
        {
            std::cerr << "Failed to create client for /raster_prob_move_service service\n";
        });

    setupClient<crawler_i2w_services::RasterProbStopRequest, crawler_i2w_services::RasterProbStopResponse>(
        runtime(),
        "/raster_prob_stop_service",
        rasterProbStopClient_,
        []
        {
            std::cerr << "Failed to create client for /raster_prob_stop_service service\n";
        });

    setupClient<crawler_i2w_services::RasterLinearActuatorMoveRequest, crawler_i2w_services::RasterLinearActuatorMoveResponse>(
        runtime(),
        "/raster_linearActuator_move_service",
        rasterLinearActuatorMoveClient_,
        []
        {
            std::cerr << "Failed to create client for /raster_linearActuator_move_service service\n";
        });

    i2w::PublisherOptions cmdVelUiPubOpt;
    cmdVelUiPubOpt.plane = i2w::EndpointPlane::Local;

    auto cmd_vel_uiPublisher = runtime().advertise<crawler_i2w_msgs::cmd_vel_ui>("/cmd_vel_ui", cmdVelUiPubOpt);

    cmd_vel_uiPublisher_ = std::move(cmd_vel_uiPublisher.value());

    connection_monitor_running_.store(true);
    connection_monitor_thread_ = std::thread([this]()
                                             {
        while (connection_monitor_running_.load())
        {
            // isConnected() may block for up to the ping timeout. Keep that
            // work out of the i2w control/service dispatch thread.
            setUiLive(isConnected());

            for (int attempt = 0;
                 attempt < 10 && connection_monitor_running_.load();
                 ++attempt)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } });

    return i2w::Ok();
}

i2w::LifecycleResult I2wControllerNode::OnTick() noexcept
{
    callUiRobotConnectionCheckService();
    publishCmd_Vel();

    if (!is_ui_live_)
    {
        cmd_vel_.linearVelocity = 0.0f;
        cmd_vel_.angularVelocity = 0.0f;
        cmd_vel_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);
        (void)cmd_velPublisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));
        //      std::cout << "Published cmd_vel: linearVelocity -> " << cmd_vel_.linearVelocity << " angularVelocity -> " << cmd_vel_.angularVelocity << std::endl;

        // std::cout << "UI is not live. Stopping the robot." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return i2w::Ok();
}

bool I2wControllerNode::isConnected()
{
    std::string host = "192.168.0.195";
    std::string command = "setsid ping -c 1 -W 1 " + host + " > /dev/null 2>&1";
    int result = system(command.c_str());
    return (result == 0);
}

void I2wControllerNode::callUiRobotConnectionCheckService()
{
    // const auto now = std::chrono::steady_clock::now();

    // // Timeout check.
    // if (waiting_for_response_ && now >= response_deadline_)
    // {
    //     waiting_for_response_ = false;
    //     setUiLive(false);
    // }

    // if (now < next_call_)
    // {
    //     return;
    // }

    // next_call_ = now + std::chrono::milliseconds(500);

    // crawler_i2w_services::UiRobotConnectionCheckRequest request;
    // request.ping = 1;
    // request.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);

    // if (McuSetting::isWatchDogEnable)
    // {
    //     const auto result = uiRobotConnectionCheckclient_.call(
    //         request, runtime().clock().now().ns,
    //         [this](const i2w::Sample<crawler_i2w_services::UiRobotConnectionCheckReponse> &sample)
    //         {
    //             if (sample.value.pong)
    //             {
    //                 waiting_for_response_ = false;
    //                 setUiLive(true);
    //             }
    //             else
    //             {
    //                 waiting_for_response_ = false;
    //                 setUiLive(false);
    //             }

    //             std::cout << "UI Robot Connection Check Service Response: " << (sample.value.pong ? "Success" : "Failure") << std::endl;
    //         });

    //     if (!result)
    //     {
    //         waiting_for_response_ = false;
    //         setUiLive(false);
    //         return;
    //     }
    // }
    // else
    // {
    //     // std::cout << "Watchdog is disabled. Skipping UI connection check." << std::endl;
    //     waiting_for_response_ = false;
    //     setUiLive(true);
    // }

    // waiting_for_response_ = true;
    // response_deadline_ = now + std::chrono::milliseconds(1000);

    // isConnected() is executed by connection_monitor_thread_. Calling it
    // here blocks OnTick() and prevents control-mode services from replying.
}

void I2wControllerNode::setUiLive(bool live)
{
    if (live == is_ui_live_.load())
    {
        return;
    }

    is_ui_live_.store(live);
    // std::cout << "UI live: " << (is_ui_live_ ? "true" : "false") << std::endl;
}

void I2wControllerNode::publishCmd_Vel_Ui(float maxLinear, float maxAngular, float linear, float angular, bool setMaxValue)
{

    crawler_i2w_msgs::cmd_vel_ui msgs;

    if (setMaxValue)
    {
        msgs.maxAngularValocity = maxLinear;
        msgs.maxAngularValocity = maxAngular;
    }
    else
    {
        msgs.angularVelocity = angular;
        msgs.linearVelocity = linear;
    }

    cmd_vel_uiPublisher_.publish(msgs, static_cast<std::int64_t>(msgs.timestamp));
}

crawler_i2w_msgs::cmd_vel I2wControllerNode::calculateTheCmd_VelToFollowWeld()
{
    constexpr float kForwardSpeed = 2.0f; // m/s along the weld
    constexpr float kKp = 20.0f;          // rad/s per metre of error (5 mm -> 0.1 rad/s)
    constexpr float kMaxTurn = 10.0f;     // rad/s, turn-rate limit
    constexpr float kDeadband = 0.05f;    // 0.5 mm: inside this, drive straight

    crawler_i2w_msgs::cmd_vel cmd_vel_to_follow_weld{};

    // Weld not found -> stop (all fields stay 0)
    if (!current_edge_status.found)
        return cmd_vel_to_follow_weld;

    // Error = weld centre relative to sensor centre (metres)
    //   error < 0 -> weld is LEFT  of centre
    //   error > 0 -> weld is RIGHT of centre
    const float error = 10 * (current_edge_status.left_x + current_edge_status.right_x) / 2.0f;

    float turn = 0.0f;
    // if (std::fabs(error) > kDeadband)
    // {
    // Positive angular_z = turn left (counter-clockwise),
    // negative angular_z = turn right.
    //   error < 0 -> -kKp * error > 0 -> turn LEFT
    //   error > 0 -> -kKp * error < 0 -> turn RIGHT
    turn = -kKp * error;

    if (turn > kMaxTurn)
        turn = kMaxTurn; // too far left -> limit
    else if (turn < -kMaxTurn)
        turn = -kMaxTurn; // too far right -> limit    }

    cmd_vel_to_follow_weld.linearVelocity = 2.0f; // forward speed along the weld
    cmd_vel_to_follow_weld.angularVelocity = turn * 2;

    return cmd_vel_to_follow_weld;
    // }
}

void I2wControllerNode::publishCmd_Vel()
{
    // cmd val correction value merge

    if (current_mode == I2wControllerNode::ControlModeType::ManualJoy)
    {
        cmd_vel_.linearVelocity = current_cmd_vel_correction.linearVelocity + current_cmd_vel_.linearVelocity;
        cmd_vel_.angularVelocity = current_cmd_vel_correction.angularVelocity + current_cmd_vel_.angularVelocity;
    }

    if (current_mode == I2wControllerNode::ControlModeType::AutoMission)
    {
        cmd_vel_.linearVelocity = current_cmd_vel_correction.linearVelocity + current_cmd_vel_misssion.linearVelocity;
        cmd_vel_.angularVelocity = current_cmd_vel_correction.angularVelocity + current_cmd_vel_misssion.angularVelocity;
    }

    if (current_mode == I2wControllerNode::ControlModeType::WeldScan)
    {

        cmd_vel_ = calculateTheCmd_VelToFollowWeld();
    }

    (void)cmd_velPublisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));

    publishCmd_Vel_Ui(0, 0, cmd_vel_.linearVelocity, cmd_vel_.angularVelocity, 0);
    // LOG_INFO("PublishCmd_Vel", std::to_string(cmd_vel_.linearVelocity) + " " + std::to_string(cmd_vel_.angularVelocity));
}

float I2wControllerNode::normalize(int16_t value, float max_output)
{

    return (static_cast<float>(value) / 32767.0f) * max_output;
}
