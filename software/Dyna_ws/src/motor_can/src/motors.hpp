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
#include "joint_msgs/msg/joint_estimates.hpp"
#include "joint_msgs/msg/joints_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "joint_msgs/msg/odrive_data.hpp"
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
    
    rclcpp::Subscription<joint_msgs::msg::Joints>::SharedPtr subscriber_joints;
    rclcpp::Publisher<joint_msgs::msg::OdriveData>::SharedPtr publisher_odrive_data;
    rclcpp::Publisher<joint_msgs::msg::Joints>::SharedPtr publisher_request;
    rclcpp::Publisher<joint_msgs::msg::JointsBool>::SharedPtr publisher_reboot;
    rclcpp::Publisher<joint_msgs::msg::JointsBool>::SharedPtr publisher_axisstate;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_state;
    rclcpp::Subscription<joint_msgs::msg::JointEstimates>::SharedPtr subscriber_estimates;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;


    void publish_joints();
    void request(const joint_msgs::msg::Joints::SharedPtr joints);
    void change_state(const std_msgs::msg::Bool::SharedPtr msg);
    void data_reception(const joint_msgs::msg::JointEstimates::SharedPtr joint);
};

#endif
