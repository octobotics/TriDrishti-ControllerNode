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

                if(sample.value.button0){
                    linearScale++;
                    std::cout<<"linearScale -> "<<std::endl;
                }
                 if(sample.value.button3){
                    linearScale--;

                }
                 if(sample.value.button1){
                    angularScale++;
                }
                 if(sample.value.button4){
                    angularScale--;
                }
                cmd_vel_.linearVelocity = -normalize(sample.value.axis2, 10*linearScale );
                cmd_vel_.angularVelocity = -normalize(sample.value.axis0, 5*angularScale );
                cmd_vel_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);
                (void)publisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));
                // std::cout<<"linearScale -> "<<linearScale<<" angularScale -> "<<angularScale<<std::endl;

            },
            opts);
        sub_ = std::move(subscription.value());

        i2w::PublisherOptions cmdVelPubOpt;
        cmdVelPubOpt.plane = i2w::EndpointPlane::Local;

        auto publisher = runtime().advertise<crawler_i2w_msgs::cmd_vel>("/cmd_vel", cmdVelPubOpt);

        publisher_ = std::move(publisher.value());

    return i2w::Ok();
}

 i2w::LifecycleResult i2wNode::OnTick() noexcept 
    {
        std::cout << "i2wNode::OnTick()" << std::endl;
        return i2w::Ok();
    }

    float normalize(int16_t value, float max_output)
{

    return (static_cast<float>(value) / 32767.0f) * max_output;
}
