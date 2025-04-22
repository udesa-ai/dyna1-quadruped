#ifndef __motors__
#define __motors__

#include "brushless.hpp"
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include "can_encoder_decoder.hpp"
#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "joint_msgs/msg/joints.hpp"
#include "joint_msgs/msg/odrive_data.hpp" 
#include "std_msgs/msg/bool.hpp"
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>

class Motors : public rclcpp::Node {
public:
    Motors();
    void reboot_odrive(std::vector<std::string> selection = {"FRshoulder","FRarm","FLarm","BRshoulder","BRarm","BLarm"});


private:
    Json::Value leg_data;
    uint8_t rrate;
    std::map<std::string, BrushlessMotor> brushless_motors;
    std::map<uint8_t, std::string> axisID;
    CanED canEncDec;
    std::string names[12];
    rclcpp::Subscription<joint_msgs::msg::Joints>::SharedPtr subscriber_joints;
    rclcpp::Publisher<joint_msgs::msg::OdriveData>::SharedPtr publisher_odrive_data;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_state;
    rclcpp::Subscription<joint_msgs::msg::Joints>::SharedPtr subscriber_max_current;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;


    void publish_joints();
    void request(const joint_msgs::msg::Joints::SharedPtr joints);
    void disengage_all();
    void engage_all();
    void change_state(const std_msgs::msg::Bool::SharedPtr msg);
    void change_max_current(const joint_msgs::msg::Joints::SharedPtr max_currents);
    // send_command(uint8_t axisID, uint8_t command, data);
};

#endif
