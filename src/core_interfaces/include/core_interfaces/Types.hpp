#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "core_interfaces/msg/heartbeat.hpp"
#include "core_interfaces/msg/killswitch.hpp"
#include "core_interfaces/msg/heartbeat_disconnect.hpp"
#include "core_interfaces/srv/heartbeat_connect.hpp"

using Heartbeat = core_interfaces::msg::Heartbeat;

struct Killswitch
{
    bool enabled;

    using SharedPtr = std::shared_ptr<Killswitch>;
};

using HeartbeatConnect = core_interfaces::srv::HeartbeatConnect;
using HeartbeatDisconnect = core_interfaces::msg::HeartbeatDisconnect;

template <>
struct rclcpp::TypeAdapter<Killswitch, core_interfaces::msg::Killswitch>
{
    using is_specialized = std::true_type;
    using custom_type = Killswitch;
    using ros_message_type = core_interfaces::msg::Killswitch;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.enabled = source.enabled;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination.enabled = source.enabled;
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Killswitch, core_interfaces::msg::Killswitch);