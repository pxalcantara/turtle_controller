#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "keyboard_msgs/msg/key.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("move_controller") {
        publisher = this->create_publisher<std_msgs::msg::String>("commands", 10);
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);    
        RCLCPP_INFO(this->get_logger(), "controller criado");

        command_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&Controller::commandCallback, this, _1));
        pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Controller::poseCallback, this, _1));

        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.linear.z = 0.0;

        initial_pose.position.x = 0.0;
        initial_pose.position.y = 0.0;
        initial_pose.position.z = 0.0;
        initial_pose.orientation.x = 0.0;
        initial_pose.orientation.y = 0.0;
        initial_pose.orientation.z = 0.0;
        initial_pose.orientation.w = 1.0;

        cmd_position_reference = 0;
        move = false;
        MOVING_THRESHOLD = 0.01;
        current_distance = 0.0;

    };

    double calculate_distance(double initial_x, double initial_y, double final_x, double final_y) {
        return std::sqrt(std::pow(final_x - initial_x, 2) + std::pow(final_y - initial_y, 2));
    }

    void stop() {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        this->cmd_vel_pub->publish(cmd_vel_msg);
    }

    void move_forward() {
        // cmd_vel_msg.linear.x = 0.2;
        // cmd_vel_msg.angular.z = 0.0;
        // this->cmd_vel_pub->publish(cmd_vel_msg);
        move_forward(0.2);
    }

    void move_forward(double speed) {
        move = true;
        // start_position = current_distance;
        cmd_vel_msg.linear.x = speed;
        cmd_vel_msg.angular.z = 0.0;
        this->cmd_vel_pub->publish(cmd_vel_msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr command_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;

    std_msgs::msg::String control_msg;
    keyboard_msgs::msg::Key command_msg;
    geometry_msgs::msg::Twist cmd_vel_msg;
    geometry_msgs::msg::Pose initial_pose;
    nav_msgs::msg::Odometry pose_msg;
    std::vector<uint16_t> commands;
    int cmd_position_reference;
    bool move;
    float MOVING_THRESHOLD;
    double current_distance;
    double start_position;
    


    void commandCallback(const keyboard_msgs::msg::Key::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), std::to_string(msg->code).c_str());
        if (msg->code == 13) {
            move = true;
            start_position = current_distance;
            return;
        }
        
        commands.push_back(msg->code);
        
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // if ( std::abs(msg->twist.twist.linear.x) > MOVING_THRESHOLD || std::abs(msg->twist.twist.angular.z) > MOVING_THRESHOLD || move) {
        //     if (!move) {
        //         start_position = current_distance;
        //     }
        //     move = true;

        // } else {
        //     move = false;
        // }
        
        if (move) {
            double moving_distance = std::abs(current_distance - start_position);
            if (msg->twist.twist.linear.x < 0) {
                move_forward(0.2);
            } else {
                move_forward(-0.1);
            }

            if (moving_distance > 1) {
                stop();
                return;
            }
            current_distance += calculate_distance(initial_pose.position.x, initial_pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.y);
            initial_pose = msg->pose.pose;
            RCLCPP_INFO_STREAM(this->get_logger(), "Distancia:" << current_distance << " delta: " << moving_distance << " init:" << start_position);
        }
        // if(msg->angular_velocity != 0 || msg->linear_velocity != 0) {
        //     if(cmd_position_reference == commands.size()) {
        //         commands.clear();
        //         cmd_position_reference = 0;
        //         move = false;
        //     }    

        //     return;
        // }
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

