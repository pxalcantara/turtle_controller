#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "keyboard_msgs/msg/key.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("move_controller") {
        publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);    
        RCLCPP_INFO(this->get_logger(), "controller criado");

        command_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&Controller::commandCallback, this, _1));

    };

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr command_sub;
    std_msgs::msg::String control_msg;
    keyboard_msgs::msg::Key command_msg;
    std::vector<uint16_t> commands;


    void commandCallback(const keyboard_msgs::msg::Key::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), std::to_string(msg->code).c_str());
        if (msg->code == 13) {
            for (auto it = commands.begin(); it != commands.end(); it++) {
                this->publishCommand(*it);
            }
            commands.clear();
            return;
        }
        
        commands.push_back(msg->code);
        
    }

    void publishCommand(const uint16_t &command){
        switch (command)
        {
        case 273:
            this->control_msg.data = "Frente";
            break;
        case 274:
            this->control_msg.data = "Tras";
            break;
        case 275: 
            this->control_msg.data = "Direita";
            break;
        case 276:
            this->control_msg.data = "Esquerda";
            break;
        default:
            return;
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Comandos: '%s', '%s'", std::to_string(command).c_str(), control_msg.data.c_str());
        this->publisher->publish(control_msg);
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}

