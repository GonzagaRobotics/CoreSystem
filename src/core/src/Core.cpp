#include "Core.hpp"

void Core::onConnect(
    const HeartbeatConnect::Request::SharedPtr request,
    HeartbeatConnect::Response::SharedPtr response)
{
    RCLCPP_INFO(get_logger(), "Got heartbeat connect request");

    if (active)
    {
        RCLCPP_WARN(get_logger(), "Already connected, ignoring request");
        response->ready = false;
        return;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Accepting connection request");
        active = true;
    }

    response->ready = true;

    lastHeartbeatTime = now();

    heartbeatInterval = std::chrono::milliseconds(request->heartbeat_interval);
    heartbeatTimeout = std::chrono::milliseconds(request->heartbeat_timeout);
    heartbeatTimeoutLimit = request->heartbeat_timeout_limit;

    heartbeatCheckTimer = create_wall_timer(
        std::chrono::milliseconds(request->heartbeat_check_interval),
        std::bind(&Core::checkHeartbeat, this));
}

void Core::onDisconnect(const HeartbeatDisconnect::SharedPtr)
{
    if (!active)
    {
        RCLCPP_WARN(get_logger(), "Received disconnect request while not connected");
        return;
    }

    RCLCPP_INFO(get_logger(), "Received disconnect request");
    active = false;
}

void Core::heartbeatSubscriberCallback(const Heartbeat::SharedPtr)
{
    if (!active)
    {
        RCLCPP_WARN(get_logger(), "Received heartbeat while not connected");
        return;
    }

    lastHeartbeatTime = now();
    missedHeartbeats = 0;
    heartbeatPublisher->publish(Heartbeat());
}

void Core::checkHeartbeat()
{
    if (!active)
    {
        return;
    }

    // Determine the time elapsed since the last heartbeat should have been received,
    // accounting for the number of missed heartbeats
    auto expectedHeartbeatTime = lastHeartbeatTime + heartbeatInterval * (missedHeartbeats + 1);

    if (now() - expectedHeartbeatTime > heartbeatTimeout)
    {
        missedHeartbeats++;

        RCLCPP_WARN(get_logger(), "Missed heartbeat #%d", missedHeartbeats);

        if (missedHeartbeats >= heartbeatTimeoutLimit)
        {
            RCLCPP_ERROR(get_logger(), "Missed too many heartbeats, disconnecting");
            active = false;
        }
    }
}

Core::Core() : rclcpp::Node("core"), heartbeatInterval(0, 0), heartbeatTimeout(0, 0)
{
    connectService = create_service<HeartbeatConnect>(
        "/heartbeat/connect",
        std::bind(&Core::onConnect, this, std::placeholders::_1, std::placeholders::_2));

    disconnectSubscription = create_subscription<HeartbeatDisconnect>(
        "/heartbeat/disconnect",
        10,
        std::bind(&Core::onDisconnect, this, std::placeholders::_1));

    heartbeatSubscription = create_subscription<Heartbeat>(
        "/heartbeat/rover",
        10,
        std::bind(&Core::heartbeatSubscriberCallback, this, std::placeholders::_1));

    heartbeatPublisher = create_publisher<Heartbeat>("/heartbeat/control", 10);

    declare_parameter("killswitch", false);

    RCLCPP_INFO(get_logger(), "Core ready");
}