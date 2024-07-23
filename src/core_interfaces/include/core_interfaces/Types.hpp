#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "core_interfaces/msg/heartbeat.hpp"
#include "core_interfaces/msg/killswitch.hpp"

struct Heartbeat
{
    std::string source;
    uint32_t id;
};

struct Killswitch
{
    bool enabled;
};

// struct HeartbeatConfigRequest
// {
//     uint32_t heartbeatInterval;
//     uint32_t heartbeatCheckInterval;
//     uint32_t heartbeatTimeout;
//     uint32_t heartbeatTimeoutLimit;
// };

// struct HeartbeatConfigResponse
// {
//     bool accepted;
// };

template <>
struct rclcpp::TypeAdapter<Heartbeat, core_interfaces::msg::Heartbeat>
{
    using is_specialized = std::true_type;
    using custom_type = Heartbeat;
    using ros_message_type = core_interfaces::msg::Heartbeat;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.source = source.source;
        destination.id = source.id;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination.source = source.source;
        destination.id = source.id;
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Heartbeat, core_interfaces::msg::Heartbeat);

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