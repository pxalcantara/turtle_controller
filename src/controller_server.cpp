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


//Ajustar modelo para corrigir sinal de velocidade com direcao de movimento
class Controller : public rclcpp::Node
{
public:
    Controller() : Node("move_controller") {
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub = this->create_publisher<turtle_controller::msg::RobotStatus>("/robot_status", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Controller::status_timer_callback, this));
        // RCLCPP_INFO(this->get_logger(), "controller criado");

        command_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&Controller::command_callback, this, _1));
        pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Controller::pose_callback, this, _1));
        robot_cmd_sub = this->create_subscription<turtle_controller::msg::RobotCmd>("/robot_cmd", 10, std::bind(&Controller::robot_cmd_callback, this, _1));
        laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::laser_scan_callback, this, _1));

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
        current_distance = 0.0;
        angular_tolerance = 0.02;
        angular_setpoints = {0, 1.57, 3.14, 4.71};
        angular_setpoint_index = 0;
        first_cmd = true;
        linear_velocity = 0.2;
        angular_velocity = 0.2;
        linear_setpoint = 1;
        angular_setpoint = angular_setpoints[1];
    };

    double calculate_distance(double initial_x, double initial_y, double final_x, double final_y) {
        return std::sqrt(std::pow(final_x - initial_x, 2) + std::pow(final_y - initial_y, 2));
    }

    void next_command() {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        this->cmd_vel_pub->publish(cmd_vel_msg);
        cmd_position_reference++;
        RCLCPP_INFO_STREAM(this->get_logger(), "Next Command" );
        rclcpp::sleep_for(std::chrono::seconds{2});
        if (cmd_position_reference == static_cast<int>(commands.size())) {
            stop();    
            return;
        }
        start_position = current_distance;
        first_cmd = true;
    }
    
    void stop () {
        move = false;
        first_cmd = true;
        cmd_position_reference = 0;
        commands.clear();
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        this->cmd_vel_pub->publish(cmd_vel_msg);
        RCLCPP_INFO_STREAM(this->get_logger(), "Stooping" );
    }

    void move_forward(float speed) {
        move = true;
        cmd_vel_msg.linear.x = speed;
        cmd_vel_msg.angular.z = 0.0;
        this->cmd_vel_pub->publish(cmd_vel_msg);
    }

    void move_forward() {
        move_forward(linear_velocity);
    }

    void rotate(float speed) {
        move = true;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = speed;
        this->cmd_vel_pub->publish(cmd_vel_msg);
    }

    void rotate() {
        rotate(angular_velocity);
    }

    double get_orientation(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
        tf2::Quaternion quaternion(
            odometry_msg->pose.pose.orientation.x,
            odometry_msg->pose.pose.orientation.y,
            odometry_msg->pose.pose.orientation.z,
            odometry_msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        if (yaw < - 0.01) {
            return yaw + 6.28;
        } else {
            return yaw;
        }
    }

    float get_angular_setpoint() {
        return angular_setpoint;
    }

    void set_angular_velocity(const float value) {
        this->angular_velocity = value;
    }

    float get_angular_velocity() {
        return this->angular_velocity;
    }

    void set_angular_setpoint(const float value) {
        this->angular_setpoint = value;
    }

    void set_linear_velocity(const float value) {
        this->linear_velocity = value;
    }

    float get_linear_velocity() {
        return this->linear_velocity;
    }

    void set_linear_setpoint(const float value) {
        this->linear_setpoint = value;
    }

    float get_linear_setpoint() {
        return this->linear_setpoint;
    }

    // MAYBE PUT THESE FUCTIONS AS PRIVATE
    void change_angular_setpoint(const bool clockwise) {
        if (clockwise) {
            angular_setpoint_index +=1;
        } else {
            angular_setpoint_index -=1;
        }

        if (angular_setpoint_index > 3) {
            angular_setpoint_index = 0;
        } else if (angular_setpoint_index < 0) {
            angular_setpoint_index = 3;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Setpoint:" << angular_setpoint_index << " / " << angular_setpoints[angular_setpoint_index]);
        set_angular_setpoint(angular_setpoints[angular_setpoint_index]);
    }


    void new_angular_setpoint (const uint16_t &command) {
        if (command != 275 && command != 276) {
            return;
        } else if (command == 275) {
            change_angular_setpoint(false);
        } else if (command == 276) {
            change_angular_setpoint(true);
        }
        return;
    }


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


    std::string to_upercase(std::string text) {
        for (auto & letter: text) letter = toupper(letter);
        return text;
    }

    void start_moving() {
        move = true;
        start_position = current_distance;
        start_orientation = current_orientation;
    }

    // center_position and sector_width in radians
    std::vector<float> get_sector_range(const float sector_center_position, const float sector_width, const std::vector<float>& range) {
        const float start_angle = sector_center_position - (sector_width/2);
        const float end_angle = sector_center_position + (sector_width/2);

        int start_index = static_cast<int>((start_angle - laser_info.angle_min ) / laser_info.angle_increment);
        int end_index = static_cast<int>((end_angle - laser_info.angle_min) / laser_info.angle_increment);
        std::vector<float> sector_range;

        for (int i = start_index; i <= end_index; ++i) {
            sector_range.push_back(range[i]);

            // Verifique se o range é válido (não infinito ou NaN)
            // if (std::isfinite(range)) {
            //     RCLCPP_INFO(this->get_logger(), "Range at index %d: %f meters", i, range);
            // }
        }
        return sector_range;
    }

    float get_sector_range_mean(const std::vector<float>& range_sector) {
        float sum = std::accumulate(range_sector.begin(), range_sector.end(), 0.0f);

        return sum / range_sector.size();        
    }


    void command_callback(const keyboard_msgs::msg::Key::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), std::to_string(msg->code).c_str());
        if (msg->code == 13) {
            if (commands.empty()) {
                return;
            }
            start_moving();
            return;
        } else if (msg->code < 273 || msg->code > 276) {
            return;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Command:" << msg->code << "cmd_size " << commands.size());
        
        commands.push_back(msg->code);
        
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
         
        current_orientation = get_orientation(msg);
        if (move) {
            if (first_cmd) {
                new_angular_setpoint(commands[cmd_position_reference]);
                first_cmd = false;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Orientation:" << current_orientation) ;
            
            double moving_distance = std::abs(current_distance - start_position);
            publish_command(commands[cmd_position_reference]);


            if (moving_distance > this->linear_setpoint && (moving_direction == FRONT || moving_direction == BACK )) {
                next_command();
                return;
            } else if ((moving_direction == RIGHT || moving_direction == LEFT )) {
                if (current_orientation >= (angular_setpoint - angular_tolerance) &&
                    current_orientation <= (angular_setpoint + angular_tolerance)) {
                    next_command();
                    return;    
                }
            }

            current_distance += calculate_distance(initial_pose.position.x, initial_pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.y);
            initial_pose = msg->pose.pose;
            RCLCPP_INFO_STREAM(this->get_logger(), "Distancia:" << current_distance );
        }

    }

    void robot_cmd_callback(const turtle_controller::msg::RobotCmd::SharedPtr msg) {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Comando:" << to_upercase(msg->direction) );
        if (!first_cmd) {
            return;
        } 

        if (msg->velocity <= 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Velocity should be greater than 0, velocity: " << msg->velocity );
            return;
        }
        
        try {
            Direction new_direction = cmd_map.at(to_upercase(msg->direction));
            commands.clear();
            if (new_direction == STOP) {
                stop();
                return;
            } else if ((new_direction == LEFT || new_direction == RIGHT) && msg->limit <= 0) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "For LEFT and RIGHT direction, limit can not be negative, limit:" << msg->limit);
                return;
            }
            commands.push_back(new_direction);
            if (new_direction == FRONT || new_direction == BACK ) {
                set_linear_velocity(msg->velocity);
                if (msg->limit < 0 ) {
                    set_linear_setpoint(UNLIMITED);    
                } else {
                    set_linear_setpoint(msg->limit);
                }
            } else {
                set_angular_velocity(msg->velocity);
                set_angular_setpoint(msg->limit);
            }
            start_moving();
        } catch(const std::exception& e) {
            std::cerr << e.what() << '\n';
            RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid command: " << msg->direction );
            return;
        }
        
    }

    void laser_scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Scan" << msg);
        laser_info.angle_increment = msg->angle_increment;
        laser_info.angle_max = msg->angle_max;
        laser_info.angle_min = msg->angle_min;
    }

    // Mudar o nome dessa funcao
    void publish_command(const uint16_t &command){
        RCLCPP_INFO_STREAM(this->get_logger(), "Executing Command:" << moving_direction);
        switch (command)
        {
        case FRONT:
            this->control_msg.data = "Frente";
            moving_direction = FRONT;
            move_forward();
            break;
        case BACK:
            this->control_msg.data = "Tras";
            moving_direction = BACK;
            move_forward(-0.2);
            break;
        case RIGHT: 
            this->control_msg.data = "Direita";
            moving_direction = RIGHT;
            rotate(-0.2);
            break;
        case LEFT:
            this->control_msg.data = "Esquerda";
            moving_direction = LEFT;
            rotate();
            break;
        default:
            break;
        }
    }

    void status_timer_callback() {
        status_msg.moving = this->move;
        status_msg.angular_velocity = this->get_angular_velocity();
        status_msg.linear_velocity = this->get_linear_velocity();
        status_msg.linear_distance = this->current_distance;
        status_msg.orientation = this->current_orientation;
        status_msg.obstacle_distance.front = 2.0;
        status_pub->publish(status_msg);
    }

    

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}

