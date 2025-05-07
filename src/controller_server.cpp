
#include "controller_server.hpp"


Controller::Controller() : Node("move_controller") {
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub = this->create_publisher<turtle_controller::msg::RobotStatus>("/robot_status", 10);
    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Controller::status_timer_callback, this));
    // RCLCPP_INFO(this->get_logger(), "controller criado");

    // command_sub = this->create_subscription<keyboard_msgs::msg::Key>("keyup", 10, std::bind(&Controller::command_callback, this, _1));
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
    // angular_setpoints = {0, 90, 180, 270};
    // angular_setpoint_index = 0;
    // first_cmd = true;
    linear_velocity = 0.2;
    angular_velocity = 0.2;
    linear_setpoint = 1;
    angular_setpoint = 0;
};

double Controller::calculate_distance(double initial_x, double initial_y, double final_x, double final_y) {
    return std::sqrt(std::pow(final_x - initial_x, 2) + std::pow(final_y - initial_y, 2));
}

float Controller::degree_to_rad(const float angle) {
    return angle * (M_PI / 180.0);
}

float Controller::rad_to_degree(const float angle) {
    return angle * (180/M_PI);
}

float Controller::normalize_angle(float angle) {
    angle = fmod(angle, 360.0);

    if (angle < 0) {
        angle += 360.0;
    }

    return angle;
}

// void Controller::next_command() {
//     cmd_vel_msg.linear.x = 0.0;
//     cmd_vel_msg.angular.z = 0.0;
//     this->cmd_vel_pub->publish(cmd_vel_msg);
//     cmd_position_reference++;
//     RCLCPP_INFO_STREAM(this->get_logger(), "Next Command" );
//     if (cmd_position_reference == static_cast<int>(commands.size())) {
//         stop();    
//         rclcpp::sleep_for(std::chrono::seconds{2});
//         return;
//     }
//     start_position = current_distance;
//     first_cmd = true;
// }

void Controller::stop () {
    move = false;
    // first_cmd = true;
    cmd_position_reference = 0;
    commands.clear();
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    this->cmd_vel_pub->publish(cmd_vel_msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Stooping" );
    rclcpp::sleep_for(std::chrono::seconds{2});
}

void Controller::move_forward(float speed) {
    move = true;
    cmd_vel_msg.linear.x = speed;
    cmd_vel_msg.angular.z = 0.0;
    this->cmd_vel_pub->publish(cmd_vel_msg);
}

void Controller::move_forward() {
    move_forward(linear_velocity);
}

void Controller::rotate(float speed) {
    move = true;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = speed;
    this->cmd_vel_pub->publish(cmd_vel_msg);
}

void Controller::rotate() {
    rotate(angular_velocity);
}

double Controller::get_orientation(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
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

float Controller::get_angular_setpoint() {
    return angular_setpoint;
}

void Controller::set_angular_velocity(const float value) {
    this->angular_velocity = value;
}

float Controller::get_angular_velocity() {
    return this->angular_velocity;
}

void Controller::set_angular_setpoint(const float value) {
    this->angular_setpoint = value;
}

void Controller::set_linear_velocity(const float value) {
    this->linear_velocity = value;
}

float Controller::get_linear_velocity() {
    return this->linear_velocity;
}

void Controller::set_linear_setpoint(const float value) {
    this->linear_setpoint = value;
}

float Controller::get_linear_setpoint() {
    return this->linear_setpoint;
}

// MAYBE PUT THESE FUCTIONS AS PRIVATE
// void Controller::change_angular_setpoint(const bool clockwise) {
//     if (clockwise) {
//         angular_setpoint_index +=1;
//     } else {
//         angular_setpoint_index -=1;
//     }

//     if (angular_setpoint_index > 3) {
//         angular_setpoint_index = 0;
//     } else if (angular_setpoint_index < 0) {
//         angular_setpoint_index = 3;
//     }
//     RCLCPP_INFO_STREAM(this->get_logger(), "Setpoint:" << angular_setpoint_index << " / " << angular_setpoints[angular_setpoint_index]);
//     float setpoint_degree = degree_to_rad(angular_setpoints[angular_setpoint_index]);
//     set_angular_setpoint(setpoint_degree);
// }


// void Controller::new_angular_setpoint (const uint16_t &command) {
//     if (command != 275 && command != 276) {
//         return;
//     } else if (command == 275) {
//         change_angular_setpoint(false);
//     } else if (command == 276) {
//         change_angular_setpoint(true);
//     }
//     return;
// }

    // center_position and sector_width in radians
std::vector<float> Controller::get_sector_range(const float sector_center_position, const float sector_width, const std::vector<float>& range) {
    const float start_angle = sector_center_position - (sector_width/2);
    const float end_angle = sector_center_position + (sector_width/2);

    int start_index = static_cast<int>((start_angle - laser_info.angle_min ) / laser_info.angle_increment);
    int end_index = static_cast<int>((end_angle - laser_info.angle_min) / laser_info.angle_increment);
    std::vector<float> sector_range;

    for (int i = start_index; i <= end_index; ++i) {
        sector_range.push_back(range[i]);

        // Verifique se o range é válido (não infinito ou NaN)
        // if (std::isfinite(range[i])) {
        //     RCLCPP_INFO_STREAM(this->get_logger(), " Valor infinito" << range[i] );
        // }
    }
    return sector_range;
}

float Controller::get_sector_range_mean(const std::vector<float>& range_sector) {
    float sum = std::accumulate(range_sector.begin(), range_sector.end(), 0.0f);

    return sum / range_sector.size();        
}

void Controller::set_laser_info(const LaserScanInfo& laser_parameters) {
    laser_info.angle_increment = laser_parameters.angle_increment;
    laser_info.angle_max = laser_parameters.angle_max;
    laser_info.angle_min = laser_parameters.angle_min;
}

LaserScanInfo Controller::get_laser_info() {
    return laser_info;
}

std::string Controller::to_upercase(std::string text) {
    for (auto & letter: text) letter = toupper(letter);
    return text;
}

void Controller::start_moving() {
    move = true;
    start_position = current_distance;
    start_orientation = current_orientation;
}



// void Controller::command_callback(const keyboard_msgs::msg::Key::SharedPtr msg) {
//     // RCLCPP_INFO(this->get_logger(), std::to_string(msg->code).c_str());
//     if (msg->code == 13) {
//         if (commands.empty()) {
//             return;
//         }
//         // start_moving();
//         return;
//     } else if (msg->code == 8) {
//         // RCLCPP_INFO_STREAM(this->get_logger(), "STOP COMMAND");
//         stop();
//         return;
//     } else if (msg->code < 273 || msg->code > 276) {
//         return;
//     }

//     turtle_controller::msg::RobotCmd::SharedPtr robot_cmd = std::make_shared<turtle_controller::msg::RobotCmd>();
//     robot_cmd->direction = "Front";
//     robot_cmd->limit = 1.0;
//     robot_cmd->velocity = linear_velocity;
//     RCLCPP_INFO_STREAM(this->get_logger(), "Setpoint UPDATED:" << get_angular_setpoint() );

//     switch (msg->code)
//     {
//     case FRONT:
//         robot_cmd->direction = "Front";
//         robot_cmd->limit = 1.0;
//         robot_cmd->velocity = linear_velocity;
//         break;
//     case BACK:
//         robot_cmd->direction = "Back";
//         robot_cmd->limit = 1.0;
//         robot_cmd->velocity = linear_velocity;
//         break;
//     case RIGHT: 
//         robot_cmd->direction = "Right";
//         robot_cmd->limit = rad_to_degree(angular_setpoint) - 90;
//         robot_cmd->velocity = angular_velocity;
//         break;
//     case LEFT:
//         robot_cmd-> direction = "left";
//         robot_cmd-> limit = rad_to_degree(angular_setpoint) + 90;
//         robot_cmd-> velocity = angular_velocity;
//         break;
//     default:
//         break;
//     }
    
//     // RCLCPP_INFO_STREAM(this->get_logger(), "Direction:" << robot_cmd->direction << "-limit " << robot_cmd->limit << "-velocity" << robot_cmd->velocity);
//     // update_robot_setpoints(robot_cmd);

//     // commands.push_back(msg->code);
//     show_commands(commands);
    
// }

void Controller::pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        
    current_orientation = get_orientation(msg);
    if (move) {
        // if (first_cmd) {
        //     new_angular_setpoint(commands[cmd_position_reference]);
        //     first_cmd = false;
        // }
        // RCLCPP_INFO_STREAM(this->get_logger(), "Orientation--:" << current_orientation) ;
        
        double moving_distance = std::abs(current_distance - start_position);
        publish_command(commands[cmd_position_reference]);


        if (moving_distance > this->linear_setpoint && (moving_direction == FRONT || moving_direction == BACK )) {
            // next_command();
            stop();
            return;
        } else if ((moving_direction == RIGHT || moving_direction == LEFT )) {
            if (current_orientation >= (angular_setpoint - angular_tolerance) &&
                current_orientation <= (angular_setpoint + angular_tolerance)) {
                // next_command();
                stop();
                return;    
            }
        }

        current_distance += calculate_distance(initial_pose.position.x, initial_pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.y);
        initial_pose = msg->pose.pose;
        // RCLCPP_INFO_STREAM(this->get_logger(), "Distancia:" << current_distance );
    }

}

void Controller::robot_cmd_callback(const turtle_controller::msg::RobotCmd::SharedPtr msg) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Comando:" << to_upercase(msg->direction) );
    try {
        Direction new_direction = cmd_map.at(to_upercase(msg->direction));
        // commands.clear();
        RCLCPP_INFO_STREAM(this->get_logger(), "COMMANDO:" << new_direction);
        if (new_direction == STOP) {
            stop();
            RCLCPP_INFO_STREAM(this->get_logger(), "parei" );
            return;
        }

        // if (!first_cmd) {
        //     return;
        // } 

        if (msg->velocity <= 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Velocity should be greater than 0, velocity: " << msg->velocity );
            return;
        }    

        if ((new_direction == LEFT || new_direction == RIGHT) && msg->limit < 0) {
            return;
            RCLCPP_ERROR_STREAM(this->get_logger(), "For LEFT and RIGHT direction, limit can not be negative, limit:" << msg->limit);
        }
        
        commands.push_back(new_direction);
        // if (new_direction == FRONT || new_direction == BACK ) {
        //     set_linear_velocity(msg->velocity);
        //     if (msg->limit < 0 ) {
        //         set_linear_setpoint(UNLIMITED);    
        //     } else {
        //         set_linear_setpoint(msg->limit);
        //     }
        // } else {
        //     set_angular_velocity(msg->velocity);
        //     set_angular_setpoint(degree_to_rad(msg->limit));
        // }
        update_robot_setpoints(msg);
        start_moving();
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid command: " << msg->direction );
        return;
    }
    
}

void Controller::laser_scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Scan" << msg);
    laser_info.angle_increment = msg->angle_increment;
    laser_info.angle_max = msg->angle_max;
    laser_info.angle_min = msg->angle_min;


    float sector_width = 20.0 * (M_PI/180);

    std::vector<float> sector_front = get_sector_range(M_PI, sector_width, msg->ranges);
    std::vector<float> sector_left = get_sector_range(-M_PI/2, sector_width, msg->ranges);
    std::vector<float> sector_right = get_sector_range(M_PI/2, sector_width, msg->ranges);
    float mean_front = get_sector_range_mean(sector_front)* 2;
    float mean_left = get_sector_range_mean(sector_left);
    float mean_right = get_sector_range_mean(sector_right);
    // RCLCPP_INFO_STREAM(this->get_logger(), "FRONT:" << mean_front << "LEFT: " << mean_left << "RIGHT: " << mean_right);
    status_msg.obstacle_distance.front = mean_front;
    status_msg.obstacle_distance.left = mean_left;
    status_msg.obstacle_distance.right = mean_right;
}
// Mudar o nome dessa funcao
void Controller::publish_command(const uint16_t &command){
    // RCLCPP_INFO_STREAM(this->get_logger(), "Executing Command:" << moving_direction);
    switch (command)
    {
    case FRONT:
        // this->control_msg.data = "Frente";
        moving_direction = FRONT;
        move_forward();
        break;
    case BACK:
        // this->control_msg.data = "Tras";
        moving_direction = BACK;
        move_forward(-0.2);
        break;
    case RIGHT: 
        // this->control_msg.data = "Direita";
        moving_direction = RIGHT;
        rotate(-0.2);
        break;
    case LEFT:
        // this->control_msg.data = "Esquerda";
        moving_direction = LEFT;
        rotate();
        break;
    default:
        break;
    }
}

void Controller::status_timer_callback() {
    status_msg.moving = this->move;
    status_msg.angular_velocity = this->get_angular_velocity();
    status_msg.linear_velocity = this->get_linear_velocity();
    status_msg.linear_distance = this->current_distance;
    status_msg.orientation = rad_to_degree(this->current_orientation);
    status_pub->publish(status_msg);
}

void Controller::show_commands(const std::vector<uint16_t>& commands) {
    std::string cmd_formatted = "";

    for (size_t i = 0; i < commands.size(); ++i) {
    std::string cmd = "";
        switch (commands[i])
        {
        case FRONT:
            cmd = "Front";
            break;
        case BACK:
            cmd = "Back";
            break;
        case RIGHT: 
            cmd = "Right";
            break;
        case LEFT:
            cmd = "Left";
            break;
        default:
            break;
        }

        cmd_formatted += "[" + cmd + "]";

        if (i != commands.size() - 1) {
            cmd_formatted += ", ";
        }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Commands: " << cmd_formatted );
}

void Controller::update_robot_setpoints(const turtle_controller::msg::RobotCmd::SharedPtr &command) {
    Direction new_direction = cmd_map.at(to_upercase(command->direction));
    
    if (new_direction == FRONT || new_direction == BACK ) {
        set_linear_velocity(command->velocity);
        if (command->limit < 0 ) {
            set_linear_setpoint(UNLIMITED);
        } else {
            set_linear_setpoint(command->limit);
        }
    } else {
        set_angular_velocity(command->velocity);
        float angle = normalize_angle(command->limit);
        // set_angular_setpoint(normalize_angle(angle));
        this->angular_setpoint = degree_to_rad(angle);
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "Setpoint UPDATED:" << get_angular_setpoint() );
    // RCLCPP_INFO_STREAM(this->get_logger(), "Direction:" << robot_cmd->direction << "-limit " << robot_cmd->limit << "-velocity" << robot_cmd->velocity);

}