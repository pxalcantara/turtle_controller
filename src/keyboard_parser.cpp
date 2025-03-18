#include "keyboard_parser.hpp"


KeyboardParser::KeyboardParser() : Node("keyboard_parser") {
    keyboard_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&KeyboardParser::keyboard_command_cb, this, _1));
    status_sub = this->create_subscription<turtle_controller::msg::RobotStatus>("/robot_status", 10, std::bind(&KeyboardParser::status_cb, this, _1));
    command_pub = this->create_publisher<turtle_controller::msg::RobotCmd>("/robot_cmd", 10);
    linear_velocity = 0.2;
    angular_velocity = 0.2;
    command_reference = 0;
    angular_setpoint = 0;
    move = false;

    robot_cmd.direction = "Front";
    robot_cmd.limit = 1.0;
    robot_cmd.velocity = linear_velocity;
};

void KeyboardParser::keyboard_command_cb(const keyboard_msgs::msg::Key::SharedPtr msg) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Key preesed:" << msg->code);
    if (msg->code == 32 && move) {
        // if (commands.empty()) {
        //     return;
        // }
        RCLCPP_INFO_STREAM(this->get_logger(), "STOP COMMAND");
        stop();
        return;
    } else if (msg->code == 13) {
        if (commands.empty()) {
            return;
        }
        publish_next_command();
        move = true;
        return;
    } else if (msg->code < FRONT || msg->code > LEFT) {
        return;
    }

    
    switch (msg->code)
    {
    case FRONT:
        robot_cmd.direction = "Front";
        robot_cmd.limit = 1.0;
        robot_cmd.velocity = linear_velocity;
        break;
        case BACK:
        robot_cmd.direction = "Back";
        robot_cmd.limit = 1.0;
        robot_cmd.velocity = linear_velocity;
        break;
        case RIGHT: 
        robot_cmd.direction = "Right";
        robot_cmd.limit = normalize_angle(angular_setpoint - 90);
        robot_cmd.velocity = angular_velocity;
        angular_setpoint = robot_cmd.limit;
        break;
        case LEFT:
        robot_cmd. direction = "Left";
        robot_cmd. limit = normalize_angle(angular_setpoint + 90);
        robot_cmd. velocity = angular_velocity;
        angular_setpoint = robot_cmd.limit;
        break;
        default:
        break;
    }
    
    // RCLCPP_INFO_STREAM(this->get_logger(), "Direction:" << robot_cmd.direction << " -limit " << robot_cmd.limit << " -velocity" << robot_cmd.velocity);
    commands.push_back(robot_cmd);
}

void KeyboardParser::status_cb(const turtle_controller::msg::RobotStatus::SharedPtr msg) {
    if (!move ) {
        return;
    }

    // RCLCPP_INFO_STREAM(this->get_logger(), "Move:" << move << commands.size());
    if (!msg->moving && !commands.empty()) {
        publish_next_command();
    } else if (!msg->moving && commands.empty()) {
        move = false;
        // RCLCPP_INFO_STREAM(this->get_logger(), "Move:" << move << "size " <<  commands.size() << "moving  " <<  msg->moving);
    }
}

void KeyboardParser::publish_next_command() {
    if (!commands.empty()) {
        robot_cmd = commands.front();
        commands.erase(commands.begin());
        command_pub->publish(robot_cmd);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Direction:" << robot_cmd.direction << " -limit " << robot_cmd.limit << " -velocity" << robot_cmd.velocity);
    }
}

void KeyboardParser::stop() {
    robot_cmd.direction = "STOP";
    robot_cmd.limit = 0;
    robot_cmd.velocity = 0;
    command_pub->publish(robot_cmd);
    commands.clear();
    move = false;
}