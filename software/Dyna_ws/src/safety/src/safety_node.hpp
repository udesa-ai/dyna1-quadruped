#ifndef __SAFETY__
#define __SAFETY__


#include "rclcpp/rclcpp.hpp"
#include "joint_msgs/msg/mini_cmd.hpp"
#include "joint_msgs/msg/joints.hpp"
#include "joint_msgs/msg/odrive_data.hpp"
#include "teleop_msgs/msg/joy_buttons.hpp"
#include "custom_sensor_msgs/msg/im_udata.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include "error_msgs/msg/error.hpp"

class SafetyPrecautions : public rclcpp::Node {
public:
    SafetyPrecautions();

    void angles_currents(joint_msgs::msg::OdriveData::SharedPtr data);
    

private:
    float alpha;
    float max_acceptable_current;
    std::vector<float> imu = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> joint_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> joint_angles_previous = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<uint8_t> encoder_check = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<float> current_check = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<float> joint_currents = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    rclcpp::Subscription<joint_msgs::msg::OdriveData>::SharedPtr subscription_joint_data;
    rclcpp::Subscription<custom_sensor_msgs::msg::IMUdata>::SharedPtr sub_imu;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::Publisher<error_msgs::msg::Error>::SharedPtr error_state;
};


#endif