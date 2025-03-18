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

float normalize_angle(float angle) {

    if (angle < 0) {
        angle += 360.0;
    } else if (angle >= 360.0) {
        angle -= 360.0;
    }

    return angle;
}

class KeyboardParser : public rclcpp::Node
{
public:
    KeyboardParser();
    
    void stop();

    

private:
    void keyboard_command_cb(const keyboard_msgs::msg::Key::SharedPtr msg);
    void status_cb(const turtle_controller::msg::RobotStatus::SharedPtr msg);
    void publish_next_command();

    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyboard_sub;
    rclcpp::Subscription<turtle_controller::msg::RobotStatus>::SharedPtr status_sub;
    rclcpp::Publisher<turtle_controller::msg::RobotCmd>::SharedPtr command_pub;

    std::vector<turtle_controller::msg::RobotCmd> commands;
    turtle_controller::msg::RobotCmd robot_cmd;
    int command_reference;
    float linear_velocity;
    float angular_velocity;
    float angular_setpoint;
    bool move;


};


#endif  // TURTLE_CONTROLLER__KEYBOARD_PARSER_HPP_