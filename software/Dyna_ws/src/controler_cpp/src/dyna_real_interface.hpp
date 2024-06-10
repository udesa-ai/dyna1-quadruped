#ifndef __dyna_real_interface__
#define __dyna_real_interface__

#include "rclcpp/rclcpp.hpp"
#include "quad_kinematics.hpp"
#include "bezier.hpp"
#include "joint_msgs/msg/mini_cmd.hpp"
#include "joint_msgs/msg/joints.hpp"
#include "joint_msgs/msg/odrive_data.hpp"
#include "teleop_msgs/msg/joy_buttons.hpp"
#include "custom_sensor_msgs/msg/im_udata.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include "trajectories.hpp"
#include <algorithm>
#include<cmath>

class RealInterface : public rclcpp::Node {
public:
    RealInterface();
    void imu_cb(const custom_sensor_msgs::msg::IMUdata::SharedPtr data);
    void update_data(joint_msgs::msg::OdriveData::SharedPtr data);
    void cmd_cb(joint_msgs::msg::MiniCmd::SharedPtr data);
    void jb_cb(teleop_msgs::msg::JoyButtons::SharedPtr data);
    void set_current(uint8_t max_current);
    void control();
    MatrixJoint get_xyz();
    void move();
    void publishall(MatrixJoint angles);

private:
    float MAX_CURRENT;
    float STEPLENGTH_SCALE;
    float Z_SCALE_CTRL;
    float RPY_SCALE;
    float SV_SCALE;
    float CHPD_SCALE;
    float YAW_SCALE;
    float BaseStepVelocity;
    float StepVelocity;
    float BaseSwingPeriod;
    float SwingPeriod;
    std::vector<float> SwingPeriod_LIMITS = {0.0f, 0.0f};
    float BaseClearanceHeight;
    float ClearanceHeight;
    float BasePenetrationDepth;
    float PenetrationDepth;
    std::vector<float> ClearanceHeight_LIMITS = {0.0f, 0.0f};
    std::vector<float> PenetrationDepth_LIMITS = {0.0f, 0.0f};
    MatrixJoint joint_angles;
    MatrixJoint joint_currents;
    bool readflag_data;
    bool config_done;
    bool current_set;
    joint_msgs::msg::MiniCmd mini_cmd;
    teleop_msgs::msg::JoyButtons jb;
    std::vector<float> imu = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    rclcpp::Time time_now;
    rclcpp::Time upt0;
    QuadModel quadKine;
    TransfDict T_bf0;
    TransfDict T_bf;
    TransfDict T_bh;
    std::vector<Eigen::Vector3f> desired = {{0.0f, 0.0f, 0.0f},
                                            {0.0f, 0.0f, 0.0f},
                                            {0.0f, 0.0f, 0.0f},
                                            {0.0f, 0.0f, 0.0f}};
    BezierGait bzg;
    rclcpp::Subscription<joint_msgs::msg::MiniCmd>::SharedPtr sub_cmd;
    rclcpp::Subscription<teleop_msgs::msg::JoyButtons>::SharedPtr sub_jb;
    bool jbreleased;
    bool sbreleased;
    bool start_movement;
    bool standing;
    uint8_t motor_states;
    uint8_t uptime;
    bool stood;
    bool descend;
    rclcpp::Subscription<joint_msgs::msg::OdriveData>::SharedPtr subscription_joint_data;
    rclcpp::Subscription<custom_sensor_msgs::msg::IMUdata>::SharedPtr sub_imu;
    rclcpp::Publisher<joint_msgs::msg::Joints>::SharedPtr ja_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_state;
    rclcpp::Publisher<joint_msgs::msg::Joints>::SharedPtr publish_max_currents;
    Trajectories traj;
    rclcpp::TimerBase::SharedPtr timer_;
    MatrixJoint adder;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    float com_offset;
};


#endif
