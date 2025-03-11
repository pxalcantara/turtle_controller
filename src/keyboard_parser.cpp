#include "keyboard_parser.hpp"


KeyboardParser::KeyboardParser() : Node("keyboard_parser") {
    keyboard_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&KeyboardParser::keyboard_command_cb, this, _1));
    command_pub = this->create_publisher<turtle_controller::msg::RobotCmd>("/robot_cmd", 10);
    linear_velocity = 0.2;
    angular_velocity = 0.2;
    command_reference = 0;
};

void KeyboardParser::keyboard_command_cb(const keyboard_msgs::msg::Key::SharedPtr msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Key preesed:" << msg->code);

}