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
    m_robotMcuConfig.ns = "";
    // m_robotMcuConfig.transport.network_profile_file =
    //     "src/TriDrishti-ControllerNode/config/ecal-network-udp.yaml";
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

i2w::LifecycleResult i2wNode::OnSetup() noexcept
{
    std::cout << "i2wNode::OnSetup()" << std::endl;
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
                    std::cout << "Speed Factor: " << speed_factor << std::endl;
                }
                if (sample.value.button0)
                {
                    speed_factor--;
                    if (speed_factor < 1)
                    {
                        speed_factor = 1;
                    }
                    std::cout << "Speed Factor: " << speed_factor << std::endl;
                }

                current_cmd_vel_.linearVelocity = -normalize(sample.value.axis2, 10 * speed_factor);
                current_cmd_vel_.angularVelocity = -normalize(sample.value.axis0, 5 * speed_factor);
                current_cmd_vel_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);
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

    i2w::PublisherOptions cmdVelPubOpt;
    cmdVelPubOpt.plane = i2w::EndpointPlane::Local;

    auto publisher = runtime().advertise<crawler_i2w_msgs::cmd_vel>("/cmd_vel", cmdVelPubOpt);

    publisher_ = std::move(publisher.value());

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

    return i2w::Ok();
}

void i2wNode::callUiRobotConnectionCheckService()
{
    const auto now = std::chrono::steady_clock::now();

    // Timeout check.
    if (waiting_for_response_ && now >= response_deadline_)
    {
        waiting_for_response_ = false;
        setUiLive(false);
    }

    if (now < next_call_)
    {
        return;
    }

    next_call_ = now + std::chrono::milliseconds(500);

    crawler_i2w_services::UiRobotConnectionCheckRequest request;
    request.ping = 1;
    request.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);

    if (McuSetting::isWatchDogEnable)
    {
        const auto result = uiRobotConnectionCheckclient_.call(
            request, runtime().clock().now().ns,
            [this](const i2w::Sample<crawler_i2w_services::UiRobotConnectionCheckReponse> &sample)
            {
                if (sample.value.pong)
                {
                    waiting_for_response_ = false;
                    setUiLive(true);
                }
                else
                {
                    waiting_for_response_ = false;
                    setUiLive(false);
                }

                std::cout << "UI Robot Connection Check Service Response: " << (sample.value.pong ? "Success" : "Failure") << std::endl;
            });

        if (!result)
        {
            waiting_for_response_ = false;
            setUiLive(false);
            return;
        }
    }
    else
    {
        // std::cout << "Watchdog is disabled. Skipping UI connection check." << std::endl;
        waiting_for_response_ = false;
        setUiLive(true);
    }

    waiting_for_response_ = true;
    response_deadline_ = now + std::chrono::milliseconds(1000);
}

void i2wNode::setUiLive(bool live)
{
    if (live == is_ui_live_)
    {
        return;
    }

    is_ui_live_ = live;
    // std::cout << "UI live: " << (is_ui_live_ ? "true" : "false") << std::endl;
}

void i2wNode::publishCmd_Vel()
{
    // cmd val correction value merge

    cmd_vel_.linearVelocity = current_cmd_vel_correction.linearVelocity + current_cmd_vel_.linearVelocity;
    cmd_vel_.angularVelocity = current_cmd_vel_correction.angularVelocity + current_cmd_vel_.angularVelocity;

    (void)publisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));

    LOG_DEBUG(
        "joy_callback",
        std::to_string(current_cmd_vel_.linearVelocity) + " " +
            std::to_string(current_cmd_vel_.angularVelocity));

    LOG_DEBUG(
        "corrected_cmd_vel_callback",
        std::to_string(current_cmd_vel_correction.linearVelocity) + " " +
            std::to_string(current_cmd_vel_correction.angularVelocity));

    LOG_DEBUG(
        "PublishCmd_Vel",
        std::to_string(cmd_vel_.linearVelocity) + " " +
            std::to_string(cmd_vel_.angularVelocity));

     std::cout << "Published cmd_vel: linearVelocity -> " << current_cmd_vel_.linearVelocity << " angularVelocity -> " << current_cmd_vel_.angularVelocity
              << "Published Corrected cmd_vel: linearVelocity -> " << current_cmd_vel_correction.linearVelocity << " angularVelocity -> " << current_cmd_vel_correction.angularVelocity

              << std::endl;
}

i2w::LifecycleResult i2wNode::OnTick() noexcept
{
    callUiRobotConnectionCheckService();
    publishCmd_Vel();

    if (!is_ui_live_)
    {
        cmd_vel_.linearVelocity = 0.0f;
        cmd_vel_.angularVelocity = 0.0f;
        cmd_vel_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);
        (void)publisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));
        //      std::cout << "Published cmd_vel: linearVelocity -> " << cmd_vel_.linearVelocity << " angularVelocity -> " << cmd_vel_.angularVelocity << std::endl;

        // std::cout << "UI is not live. Stopping the robot." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return i2w::Ok();
}

float i2wNode::normalize(int16_t value, float max_output)
{

    return (static_cast<float>(value) / 32767.0f) * max_output;
}
