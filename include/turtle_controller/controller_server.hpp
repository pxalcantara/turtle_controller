#include <memory>
#include <vector>
#include <cmath>
#include <chrono>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "keyboard_msgs/msg/key.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlesim/msg/pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "turtle_controller/msg/robot_status.hpp"
#include "turtle_controller/msg/robot_cmd.hpp"

using std::placeholders::_1;

const float UNLIMITED = 10000.0;

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

struct LaserScanInfo {
    float angle_min;
    float angle_max;
    float angle_increment;
};

class Controller : public rclcpp::Node
{
public:
    Controller();

    double calculate_distance(double initial_x, double initial_y, double final_x, double final_y);

    void next_command();
    
    void stop(); 

    void move_forward(float speed);

    void move_forward();

    void rotate(float speed);

    void rotate(); 

    double get_orientation(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);

    float get_angular_setpoint();

    void set_angular_velocity(const float value);

    float get_angular_velocity(); 

    void set_angular_setpoint(const float value);

    void set_linear_velocity(const float value);

    float get_linear_velocity(); 

    void set_linear_setpoint(const float value);

    float get_linear_setpoint();

    // MAYBE PUT THESE FUCTIONS AS PRIVATE
    void change_angular_setpoint(const bool clockwise);

    void new_angular_setpoint (const uint16_t &command);

        // center_position and sector_width in radians
    std::vector<float> get_sector_range(const float sector_center_position, const float sector_width, const std::vector<float>& range);

    float get_sector_range_mean(const std::vector<float>& range_sector);

    void set_laser_info(const LaserScanInfo & laser_parameters);
    
    LaserScanInfo get_laser_info();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<turtle_controller::msg::RobotStatus>::SharedPtr status_pub;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr command_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
    rclcpp::Subscription<turtle_controller::msg::RobotCmd>::SharedPtr robot_cmd_sub;

    std_msgs::msg::String control_msg;
    geometry_msgs::msg::Twist cmd_vel_msg;
    turtle_controller::msg::RobotStatus status_msg;
    keyboard_msgs::msg::Key command_msg;
    geometry_msgs::msg::Pose initial_pose;
    nav_msgs::msg::Odometry pose_msg;

    std::vector<uint16_t> commands;
    int cmd_position_reference;
    bool move;
    float linear_setpoint;
    double current_distance;
    double start_position;
    float angular_tolerance;
    double start_orientation;
    double current_orientation;
    float angular_setpoint;
    std::vector<float> angular_setpoints;
    int angular_setpoint_index;
    bool first_cmd;
    Direction moving_direction;
    float linear_velocity;
    float angular_velocity;
    rclcpp::TimerBase::SharedPtr timer;
    LaserScanInfo laser_info;


    std::string to_upercase(std::string text);

    void start_moving();

    void command_callback(const keyboard_msgs::msg::Key::SharedPtr msg);

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void robot_cmd_callback(const turtle_controller::msg::RobotCmd::SharedPtr msg);

    void laser_scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Mudar o nome dessa funcao
    void publish_command(const uint16_t &command);

    void status_timer_callback() ;    

};
