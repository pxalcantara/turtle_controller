#ifndef TURTLE_CONTROLLER__KEYBOARD_PARSER_HPP_
#define TURTLE_CONTROLLER__KEYBOARD_PARSER_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "keyboard_msgs/msg/key.hpp"
#include "turtle_controller/msg/robot_status.hpp"
#include "turtle_controller/msg/robot_cmd.hpp"

using std::placeholders::_1;


enum Direction {
    FRONT = 273,
    BACK = 274,
    RIGHT = 275,
    LEFT = 276,
    STOP = 0,
    UNKNOWN,
};

static const std::map<std::string, Direction> cmd_map = {
    {"FRONT", FRONT},
    {"BACK", BACK},
    {"RIGHT", RIGHT},
    {"LEFT", LEFT},
    {"STOP", STOP}
};

class KeyboardParser : public rclcpp::Node
{
public:
    KeyboardParser();

    // void next_command();

    

private:
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyboard_sub;
    rclcpp::Publisher<turtle_controller::msg::RobotCmd>::SharedPtr command_pub;

    std::vector<turtle_controller::msg::RobotCmd> commands;
    int command_reference;
    float linear_velocity;
    float angular_velocity;

    void keyboard_command_cb(const keyboard_msgs::msg::Key::SharedPtr msg);

};


#endif  // TURTLE_CONTROLLER__KEYBOARD_PARSER_HPP_