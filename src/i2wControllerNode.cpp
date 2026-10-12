#include "i2wControllerNode.hpp"
#include <iostream>
#include "mcuLogger.hpp"

I2wControllerNode::I2wControllerNode(i2w::Config config) : i2w::SystemBase(std::move(config)), Controller()
{
    LOG_INFO("I2wControllerNode", "I2wControllerNode created");
}

I2wControllerNode::~I2wControllerNode()
{
    connection_monitor_running_.store(false);

    connection_monitor_thread_.join();
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

    return i2w::Ok();
}

void I2wControllerNode::move_robot()
{
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

// ---------------------------------------------------------------------------
// Weld-following controller: /edge_status  ->  cmd_vel
//
//  Objective 1 : weld centre at laser x = 0          (centre error -> 0)
//  Objective 2 : measured width == true width        (heading error -> 0)
//                w_meas = w_true / cos(theta)  (10 mm reads 18 mm at ~56 deg)
//
//  ALIGN : rotate in place, turn whichever way makes the width shrink
//  TRACK : drive at scan speed, omega = -( sat(Kp*e + Ki*int e) + Kh*theta )
//  LOST  : edge missing longer than LOST_TIMEOUT -> stop
//
//  Conventions: linear_x forward +, angular_z CCW (left turn) +  (REP-103).
//  Internally: weld to the robot's RIGHT = positive lateral value.
//
//  Needs: #include <algorithm> <chrono> <cmath> <cstdio> <deque> <utility>
// ---------------------------------------------------------------------------



crawler_i2w_msgs::cmd_vel I2wControllerNode::calculateTheCmd_VelToFollowWeld(crawler_i2w_msgs::edge_status current_edge_status)
{

    auto make_cmd = [&](double v, double w)
    {
        
        I2wControllerNode::st.last_v = v;
        I2wControllerNode::st.last_w = w;
        crawler_i2w_msgs::cmd_vel cmd{};
        cmd.linearVelocity = static_cast<float>(v); // <-- rename to your cmd_vel fields
        cmd.angularVelocity = static_cast<float>(w);
        std::printf("[cmd_vel] %-5s linear.x=%+.4f angular.z=%+.4f | width=%.2fmm centre=%+.2fmm theta=%+.1fdeg\n",
                    mode_name[int(I2wControllerNode::st.mode)], v, w, I2wControllerNode::st.width_f * 1e3, I2wControllerNode::st.centre_f * 1e3, I2wControllerNode::st.theta * 57.2958);
        return cmd;
    };

    auto enter_mode = [&](Mode m)
    {
        if (m == I2wControllerNode::st.mode)
            return;
        std::printf("[weld] %s -> %s (width=%.2fmm centre=%+.2fmm)\n",
                    mode_name[int(I2wControllerNode::st.mode)], mode_name[int(m)], I2wControllerNode::st.width_f * 1e3, I2wControllerNode::st.centre_f * 1e3);
        I2wControllerNode::st.mode = m;
        if (m == Mode::ALIGN)
        {
            I2wControllerNode::st.align_ref = I2wControllerNode::st.width_f;
            I2wControllerNode::st.align_ok = 0;
            // weld to the right (theta>0) -> turn right (negative omega)
            I2wControllerNode::st.align_dir = I2wControllerNode::st.last_theta_sign != 0 ? -double(I2wControllerNode::st.last_theta_sign) : 1.0;
        }
        if (m == Mode::TRACK)
        {
            
            I2wControllerNode::st.hist.clear();
            I2wControllerNode::st.integ = 0.0;
        }
    };

    // ============================ TIME & DEAD RECKONING =====================
    const double now = std::chrono::duration<double>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
    const double dt = I2wControllerNode::st.has_prev ? std::clamp(now - I2wControllerNode::st.t_prev, 0.0, 0.2) : 0.0;
    I2wControllerNode::st.has_prev = true;
    I2wControllerNode::st.t_prev = now;
    I2wControllerNode::st.s += I2wControllerNode::st.last_v * dt; // replace with wheel odometry if available
    I2wControllerNode::st.psi += I2wControllerNode::st.last_w * dt;

    // ============================ EDGE LOST =================================
    const auto &m = current_edge_status;
    if (!m.found || m.right_x <= m.left_x)
    {
        const bool brief = (I2wControllerNode::st.mode == Mode::ALIGN || I2wControllerNode::st.mode == Mode::TRACK) &&
                           (now - I2wControllerNode::st.t_last_found < LOST_TIMEOUT);
        if (brief)
            return make_cmd(I2wControllerNode::st.last_v, I2wControllerNode::st.last_w); // short dropout: hold
        if (I2wControllerNode::st.mode != Mode::WAIT)
            enter_mode(Mode::LOST);
        I2wControllerNode::st.filt_init = false;
        return make_cmd(0.0, 0.0);
    }
    I2wControllerNode::st.t_last_found = now;

    // ============================ MEASUREMENT ===============================
    const double sx = SENSOR_X_RIGHT ? 1.0 : -1.0;
    const double centre = sx * 0.5 * (double(m.left_x) + double(m.right_x));
    const double width = std::fabs(double(m.right_x) - double(m.left_x));

    if (!I2wControllerNode::st.filt_init)
    {
        I2wControllerNode::st.centre_f = centre;
        I2wControllerNode::st.width_f = width;
        I2wControllerNode::st.filt_init = true;
    }
    else
    {
        I2wControllerNode::st.centre_f += EMA_ALPHA * (centre - I2wControllerNode::st.centre_f);
        I2wControllerNode::st.width_f += EMA_ALPHA * (width - I2wControllerNode::st.width_f);
    }
    const double width_err = I2wControllerNode::st.width_f - WELD_WIDTH;

    if (I2wControllerNode::st.mode == Mode::WAIT || I2wControllerNode::st.mode == Mode::LOST)
        enter_mode(width_err > ALIGN_TOL ? Mode::ALIGN : Mode::TRACK);

    // ============================ ALIGN =====================================
    if (I2wControllerNode::st.mode == Mode::ALIGN)
    {
        // width rose above its running minimum -> we passed it / wrong way: flip
        if (I2wControllerNode::st.width_f > I2wControllerNode::st.align_ref + WIDTH_NOISE)
        {
            I2wControllerNode::st.align_dir = -I2wControllerNode::st.align_dir;
            I2wControllerNode::st.align_ref = I2wControllerNode::st.width_f;
        }
        else if (I2wControllerNode::st.width_f < I2wControllerNode::st.align_ref)
        {
            I2wControllerNode::st.align_ref = I2wControllerNode::st.width_f;
        }

        if (width_err < ALIGN_TOL)
        {
            if (++I2wControllerNode::st.align_ok >= ALIGN_HOLD)
                enter_mode(Mode::TRACK); // fall through
        }
        else
        {
            I2wControllerNode::st.align_ok = 0;
        }
        if (I2wControllerNode::st.mode == Mode::ALIGN)
            return make_cmd(0.0, I2wControllerNode::st.align_dir * ALIGN_OMEGA);
    }

    // ============================ TRACK =====================================
    if (width_err > REALIGN_TOL)
    {
        enter_mode(Mode::ALIGN);
        return make_cmd(0.0, I2wControllerNode::st.align_dir * ALIGN_OMEGA);
    }

    // Heading estimate.
    // Coarse magnitude from width: |theta| = acos(w_true / w_meas)
    const double theta_w = std::acos(std::clamp(WELD_WIDTH / I2wControllerNode::st.width_f, 0.0, 1.0));

    // Fine, signed: slope of centre vs travelled distance (tan theta).
    // Subtract SENSOR_OFFSET*psi so our own rotation (laser sweeping sideways)
    // isn't mistaken for heading error.
    I2wControllerNode::st.hist.emplace_back(I2wControllerNode::st.s, I2wControllerNode::st.centre_f - SENSOR_OFFSET * I2wControllerNode::st.psi);
    while (I2wControllerNode::st.hist.size() > SLOPE_WINDOW)
        I2wControllerNode::st.hist.pop_front();

    bool slope_ok = false;
    double slope = 0.0;
    if (I2wControllerNode::st.hist.size() >= 3 && I2wControllerNode::st.hist.back().first - I2wControllerNode::st.hist.front().first >= SLOPE_MIN_DS)
    {
        double ms = 0.0, mq = 0.0;
        for (const auto &h : I2wControllerNode::st.hist)
        {
            ms += h.first;
            mq += h.second;
        }
        ms /= I2wControllerNode::st.hist.size();
        mq /= I2wControllerNode::st.hist.size();
        double num = 0.0, den = 0.0;
        for (const auto &h : I2wControllerNode::st.hist)
        {
            num += (h.first - ms) * (h.second - mq);
            den += (h.first - ms) * (h.first - ms);
        }
        if (den > 1e-12)
        {
            slope = num / den;
            slope_ok = true;
        }
    }

    if (slope_ok)
    {
        const double theta_s = std::atan(slope);
        I2wControllerNode::st.theta = (std::fabs(theta_s) < HEADING_SWITCH) ? theta_s                          // fine
                                                         : std::copysign(theta_w, theta_s); // coarse
        I2wControllerNode::st.last_theta_sign = (I2wControllerNode::st.theta > 0.0) - (I2wControllerNode::st.theta < 0.0);
    }
    else
    {
        I2wControllerNode::st.theta = 0.0; // not enough travel yet -> centre term only
    }

    // Integral on centre (only near the seam -> no windup on long approaches)
    if (std::fabs(I2wControllerNode::st.centre_f) < I_BAND)
        I2wControllerNode::st.integ = std::clamp(I2wControllerNode::st.integ + I2wControllerNode::st.centre_f * dt,
                              -I_OMEGA_MAX / K_CENTER_I, I_OMEGA_MAX / K_CENTER_I);

    // Centre term capped -> bounded approach angle, width never blows up
    const double lat = std::clamp(K_CENTER * I2wControllerNode::st.centre_f + K_CENTER_I * I2wControllerNode::st.integ,
                                  -LAT_OMEGA_MAX, LAT_OMEGA_MAX);

    const double omega = std::clamp(-(lat + K_HEADING * I2wControllerNode::st.theta), -OMEGA_MAX, OMEGA_MAX);
    const double v = V_SCAN * std::max(V_MIN_SCALE, std::cos(theta_w)); // slow when misaligned

    return make_cmd(v, omega);
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
    // LOG_INFO("PublishCmd_Vel", std::to_string(cmd_vel_.linearVelocity) + " " + std::to_string(cmd_vel_.angularVelocity));
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
