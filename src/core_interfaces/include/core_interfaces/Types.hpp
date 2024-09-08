#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "core_interfaces/msg/heartbeat.hpp"
#include "core_interfaces/msg/heartbeat_disconnect.hpp"
#include "core_interfaces/srv/heartbeat_connect.hpp"

using Heartbeat = core_interfaces::msg::Heartbeat;
using HeartbeatConnect = core_interfaces::srv::HeartbeatConnect;
using HeartbeatDisconnect = core_interfaces::msg::HeartbeatDisconnect;