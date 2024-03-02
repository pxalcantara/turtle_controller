#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "keyboard_msgs/msg/key.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("move_controller") {
        publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);    
        RCLCPP_INFO(this->get_logger(), "controller criado");

        command_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&Controller::commandCallback, this, _1));
        pose_sub = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&Controller::poseCallback, this, _1));

        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.linear.z = 0.0;

        cmd_position_reference = 0;
        move = false;

    };

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr command_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;

    std_msgs::msg::String control_msg;
    keyboard_msgs::msg::Key command_msg;
    geometry_msgs::msg::Twist cmd_vel_msg;
    turtlesim::msg::Pose pose_msg;
    std::vector<uint16_t> commands;
    int cmd_position_reference;
    bool move;


    void commandCallback(const keyboard_msgs::msg::Key::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), std::to_string(msg->code).c_str());
        if (msg->code == 13) {
            move = true;
            return;
        }
        
        commands.push_back(msg->code);
        
    }

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
        if(msg->angular_velocity != 0 || msg->linear_velocity != 0) {
            if(cmd_position_reference == commands.size()) {
                commands.clear();
                cmd_position_reference = 0;
                move = false;
            }    

            return;
        }
        if(!commands.empty() && move) {
            // RCLCPP_INFO(this->get_logger(), std::to_string(commands[cmd_position_reference]).c_str());
            publishCommand(commands[cmd_position_reference]);
            cmd_position_reference++;
        }
    }

    void publishCommand(const uint16_t &command){
        switch (command)
        {
        case 273:
            this->control_msg.data = "Frente";
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_msg.linear.x = 1.0;
            break;
        case 274:
            this->control_msg.data = "Tras";
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_msg.linear.x = -1.0;
            break;
        case 275: 
            this->control_msg.data = "Direita";
            cmd_vel_msg.angular.z = -1.55;
            cmd_vel_msg.linear.x = 0.0;
            break;
        case 276:
            this->control_msg.data = "Esquerda";
            cmd_vel_msg.angular.z = 1.55;
            cmd_vel_msg.linear.x = 0.0;
            break;
        default:
            return;
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Comandos: '%s', '%s'", std::to_string(command).c_str(), control_msg.data.c_str());
        this->publisher->publish(control_msg);
        this->cmd_vel_pub -> publish(cmd_vel_msg);
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}

