#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "keyboard_msgs/msg/key.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("move_controller") {
        this->msg.data = "turtle_controller";
        publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);    
        RCLCPP_INFO(this->get_logger(), "controller criado");
        this->publisher->publish(msg);

        command_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&Controller::commandCallback, this, _1));

    };

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr command_sub;
    std_msgs::msg::String msg;
    keyboard_msgs::msg::Key command_msg;

    void commandCallback(const keyboard_msgs::msg::Key::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), std::to_string(msg->code).c_str());
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}

