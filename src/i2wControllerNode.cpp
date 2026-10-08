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

    setupSubscriber();

    setupPublisher();

    return i2w::Ok();
}

i2w::LifecycleResult I2wControllerNode::OnTick() noexcept
{
    callUiRobotConnectionCheckService();

    move_robot();

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

void I2wControllerNode::move_robot(){
    current_edge_status = detectEdge(current_laser_profile.points);
    cmd_vel_ = calculateTheCmd_VelToFollowWeld(current_edge_status);
    publishCmd_Vel(cmd_vel_);
    publisher_edge_status(current_edge_status);
}


void I2wControllerNode::setupSubscriber()
{
    i2w::SubscriptionOptions sub_options;
    sub_options.plane = i2w::EndpointPlane::Local;
    sub_options.reliability = i2w::Reliability::BestEffort;
    sub_options.queue_depth = 32;
    sub_options.overflow_policy = i2w::OverflowPolicy::DropOldest;

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
        sub_options);
    sub_ = std::move(subscription.value());

    auto cmd_vel_correction_sub = runtime().subscribe<crawler_i2w_msgs::cmd_vel>(
        "/cmd_vel_correction",
        [this](const i2w::Sample<crawler_i2w_msgs::cmd_vel> &sample)
        {
            current_cmd_vel_correction = sample.value;
        },
        sub_options);
    cmd_vel_correction_sub_ = std::move(cmd_vel_correction_sub.value());

    // sub edge status

    auto edge_status_sub = runtime().subscribe<crawler_i2w_msgs::edge_status>(
        "/edge_status",
        [this](const i2w::Sample<crawler_i2w_msgs::edge_status> &sample)
        {
            current_edge_status = sample.value;
        },
        sub_options);
    edge_status_sub_ = std::move(edge_status_sub.value());

    // mission cmd_vel

    auto cmd_vel_mission_sub = runtime().subscribe<crawler_i2w_msgs::cmd_vel>(
        "/mission/cmd_vel",
        [this](const i2w::Sample<crawler_i2w_msgs::cmd_vel> &sample)
        {
            current_cmd_vel_misssion = sample.value;
        },
        sub_options);
    cmd_vel_mission_sub_ = std::move(cmd_vel_mission_sub.value());

    // laser profile sub
    auto laser_profile_sub = runtime().subscribe<crawler_i2w_msgs::I2wScanControlProfile>(
        "profile",
        [this](const i2w::Sample<crawler_i2w_msgs::I2wScanControlProfile> &sample)
        {
            current_laser_profile = sample.value;

        },
        sub_options);
    laser_profile_sub_ = std::move(laser_profile_sub.value());
}

void I2wControllerNode::setupPublisher()
{
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

crawler_i2w_msgs::cmd_vel I2wControllerNode::calculateTheCmd_VelToFollowWeld(crawler_i2w_msgs::edge_status current_edge_status)
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

    // std::cout << "Weld follow: error = " << error << " turn = " << turn << std::endl;

    return cmd_vel_to_follow_weld;
    // }
}

void I2wControllerNode::publishCmd_Vel(crawler_i2w_msgs::cmd_vel &cmd_vel_)
{
    // cmd val correction value merge

    // if (current_mode == I2wControllerNode::ControlModeType::ManualJoy)
    // {
    //     cmd_vel_.linearVelocity = current_cmd_vel_correction.linearVelocity + current_cmd_vel_.linearVelocity;
    //     cmd_vel_.angularVelocity = current_cmd_vel_correction.angularVelocity + current_cmd_vel_.angularVelocity;
    // }

    // if (current_mode == I2wControllerNode::ControlModeType::AutoMission)
    // {
    //     cmd_vel_.linearVelocity = current_cmd_vel_correction.linearVelocity + current_cmd_vel_misssion.linearVelocity;
    //     cmd_vel_.angularVelocity = current_cmd_vel_correction.angularVelocity + current_cmd_vel_misssion.angularVelocity;
    // }

    // if (current_mode == I2wControllerNode::ControlModeType::WeldScan)
    // {

        // publisher_edge_status(current_laser_profile);
       
    // }

    (void)cmd_velPublisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));

    publishCmd_Vel_Ui(0, 0, cmd_vel_.linearVelocity, cmd_vel_.angularVelocity, 0);
    LOG_INFO("PublishCmd_Vel", std::to_string(cmd_vel_.linearVelocity) + " " + std::to_string(cmd_vel_.angularVelocity));
}

float I2wControllerNode::normalize(int16_t value, float max_output)
{

    return (static_cast<float>(value) / 32767.0f) * max_output;
}

crawler_i2w_msgs::edge_status I2wControllerNode::detectEdge(const std::array<crawler_i2w_msgs::ScanControlPoint, crawler_i2w_msgs::kI2wScanControlMaxPoints> &pts)
{
    crawler_i2w_msgs::edge_status e;


    // Base level = median Z of all valid points
    std::vector<float> zs;

    zs.reserve(pts.size());
    for (const auto &p : pts)
        if (p.valid)
            zs.push_back(p.z_m);
    if (zs.size() < 10)
        return e;
    std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
    e.baseline_z = zs[zs.size() / 2];

    // Find the longest run of consecutive "high" points
    int best_start = -1, best_end = -1, best_len = 0;
    int cur_start = -1, cur_end = -1, cur_len = 0;

    for (int i = 0; i < static_cast<int>(pts.size()); ++i)
    {
        const auto &p = pts[i];
        if (!p.valid)
            continue; // invalid points don't break the run
        const bool high = std::fabs(e.baseline_z - p.z_m) > kEdgeThreshold;
        if (high)
        {
            if (cur_len == 0)
                cur_start = i;
            cur_end = i;
            ++cur_len;
            if (cur_len > best_len)
            {
                best_len = cur_len;
                best_start = cur_start;
                best_end = cur_end;
            }
        }
        else
        {
            cur_len = 0;
        }
    }
    if (best_len < kEdgeMinRun)
        return e;

    // Left/right edge = smallest/largest X of the high run
    e.left_x = 1e9f;
    e.right_x = -1e9f;
    for (int i = best_start; i <= best_end; ++i)
    {
        const auto &p = pts[i];
        if (!p.valid)
            continue;
        const float d = std::fabs(e.baseline_z - p.z_m);
        if (d <= kEdgeThreshold)
            continue;
        e.left_x = std::min(e.left_x, p.x_m);
        e.right_x = std::max(e.right_x, p.x_m);
        e.height = std::max(e.height, d);
    }

    if (e.left_x > -activeLength && e.right_x < activeLength)
    {
        e.found = true;
        return e;
    }

    return e;
}

void I2wControllerNode::publisher_edge_status(const crawler_i2w_msgs::edge_status &current_edge_status)
{
    edge_statusPublisher_.publish(current_edge_status, runtime().clock().now().ns);

    // LOG_INFO("Edge Status Published", "Found = " + std::to_string(current_edge_status.found) +
    //                                 ", Left X = " + std::to_string(current_edge_status.left_x) +
    //                                 ", Right X = " + std::to_string(current_edge_status.right_x) +
    //                                 ", Baseline Z = " + std::to_string(current_edge_status.baseline_z) +
    //                                 ", Height = " + std::to_string(current_edge_status.height));
  
}
