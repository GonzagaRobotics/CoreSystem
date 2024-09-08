#pragma once

#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "core_interfaces/Types.hpp"

class Core : public rclcpp::Node
{
private:
    rclcpp::Duration heartbeatInterval;
    rclcpp::Duration heartbeatTimeout;
    uint32_t heartbeatTimeoutLimit;

    bool active = false;
    rclcpp::Time lastHeartbeatTime;
    uint32_t missedHeartbeats = 0;

    rclcpp::Service<HeartbeatConnect>::SharedPtr connectService;

    void onConnect(
        const HeartbeatConnect::Request::SharedPtr request,
        HeartbeatConnect::Response::SharedPtr response);

    rclcpp::Subscription<HeartbeatDisconnect>::SharedPtr disconnectSubscription;

    void onDisconnect(const HeartbeatDisconnect::SharedPtr);

    rclcpp::Subscription<Heartbeat>::SharedPtr heartbeatSubscription;
    rclcpp::Publisher<Heartbeat>::SharedPtr heartbeatPublisher;

    void heartbeatSubscriberCallback(const Heartbeat::SharedPtr);

    rclcpp::TimerBase::SharedPtr heartbeatCheckTimer;

    void checkHeartbeat();

public:
    Core();
};