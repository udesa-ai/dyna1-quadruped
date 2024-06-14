#ifndef DYNA_STATE_MACHINE
#define DYNA_STATE_MACHINE

#include <math.h>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "joint_msgs/msg/mini_cmd.hpp"
#include <chrono>


class StateMachine : public rclcpp::Node {
    public:
        StateMachine();

        constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-1);

        void update_command(const double & vx, const double & vy, const double & z,
                            const double & w, const double & wx, const double & wy);

        void switch_movement();

        void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr tw);

        void swm_callback(const std_msgs::msg::Bool::SharedPtr msg);

        void estop_callback(const std_msgs::msg::Bool::SharedPtr estop);
    
    private:

        double filter(double value, double previous);

        enum Motion {Go, Stop};
        enum Movement {Stepping, Viewing};

        void process();

        struct SpotCommand
        {
            Motion motion;
            Movement movement;
            double x_velocity;
            double y_velocity;
            double rate;
            double roll;
            double pitch;
            double yaw;
            double z;
            double faster;
            double slower;
        };
        
        SpotCommand cmd;

        double alpha;

        bool teleop_flag;
        bool motion_flag;
        bool ESTOP;
        rclcpp::Time current_time;
        rclcpp::Time last_time;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr switch_movement_pub;
        rclcpp::Publisher<joint_msgs::msg::MiniCmd>::SharedPtr mini_pub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_movement_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub;
        double timeout;
        joint_msgs::msg::MiniCmd mini_cmd;
        rclcpp::TimerBase::SharedPtr timer_;
        bool estop_pressed;
};

#endif