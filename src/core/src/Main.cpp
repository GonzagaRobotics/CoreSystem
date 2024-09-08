#include "Core.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto core = std::make_shared<Core>();
    rclcpp::spin(core);

    rclcpp::shutdown();
}