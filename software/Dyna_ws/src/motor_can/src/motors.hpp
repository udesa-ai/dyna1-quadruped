#ifndef __motors__
#define __motors__

#include "brushless.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "joint_msgs/msg/joints.hpp"
#include "std_msgs/msg/bool.hpp"
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>

class Motors : public rclcpp::Node {
public:
    Motors();
    void reboot_odrive();
    void declare_leg_config(const std::string &leg_name);


private:
    uint8_t rrate;
    std::map<std::string, BrushlessMotor> brushless_motors;
    std::map<uint8_t, std::string> axisID;
    std::string names[12];
    
    
    
    // rclcpp::Subscription<joint_msgs::msg::Joints>::SharedPtr subscriber_joints;
    // rclcpp::Publisher<joint_msgs::msg::OdriveData>::SharedPtr publisher_odrive_data;
    // rclcpp::Publisher<joint_msgs::msg::CanFloat>::SharedPtr publisher_request;
    // rclcpp::Publisher<joint_msgs::msg::CanFloat>::SharedPtr publisher_maxC;
    // rclcpp::Publisher<joint_msgs::msg::CanInt>::SharedPtr publisher_reboot;
    // rclcpp::Publisher<joint_msgs::msg::CanInt>::SharedPtr publisher_axisstate;
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_state;
    // rclcpp::Subscription<joint_msgs::msg::Joints>::SharedPtr subscriber_max_current;
    // rclcpp::Subscription<joint_msgs::msg::Estimates>::SharedPtr subscriber_estimates;
    // rclcpp::Subscription<joint_msgs::msg::Current>::SharedPtr subscriber_current;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;


    void publish_joints();
    void request(const joint_msgs::msg::Joints::SharedPtr joints);
    void change_state(const std_msgs::msg::Bool::SharedPtr msg);
    void change_max_current(const joint_msgs::msg::Joints::SharedPtr max_currents);
    void encoder_reception(const joint_msgs::msg::Estimates::SharedPtr joint);
    void current_reception(const joint_msgs::msg::Current::SharedPtr joint);
    void state_change(uint8_t state);
    // send_command(uint8_t axisID, uint8_t command, data);
};

#endif
