#ifndef __dyna_real_interface__
#define __dyna_real_interface__

#include "rclcpp/rclcpp.hpp"
#include "quad_kinematics.hpp"
#include "bezier.hpp"
#include "joint_msgs/msg/mini_cmd.hpp"
#include "joint_msgs/msg/joints.hpp"
#include "joint_msgs/msg/odrive_data.hpp"
#include "joint_msgs/msg/neural_input.hpp"
#include "teleop_msgs/msg/joy_buttons.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "error_msgs/msg/error.hpp"
#include <chrono>
#include "trajectories.hpp"
#include <algorithm>
#include <cmath>

class RealInterface : public rclcpp::Node {
public:
    RealInterface();
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr data);
    void update_data(joint_msgs::msg::OdriveData::SharedPtr data);
    void cmd_cb(joint_msgs::msg::MiniCmd::SharedPtr data);
    void jb_cb(teleop_msgs::msg::JoyButtons::SharedPtr data);
    void set_current(uint8_t max_current);
    void control();
    MatrixJoint get_xyz();
    void move();
    void move_nn();
    void publishall(MatrixJoint angles);
    void error_update(error_msgs::msg::Error::SharedPtr data);

private:
    uint8_t ERROR_STATE = 0;
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
    MatrixJoint joint_velocities_rpm;
    MatrixJoint joint_currents;
    bool readflag_data;
    bool config_done;
    bool current_set;
    joint_msgs::msg::MiniCmd mini_cmd;
    teleop_msgs::msg::JoyButtons jb;
    std::vector<float> imu = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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
    bool nnreleased;
    bool sbreleased;
    bool start_movement;
    bool standing;
    uint8_t motor_states;
    uint8_t uptime;
    bool stood;
    bool descend;
    bool nn_state;
    rclcpp::Subscription<joint_msgs::msg::OdriveData>::SharedPtr subscription_joint_data;
    rclcpp::Subscription<error_msgs::msg::Error>::SharedPtr errors_data;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_velocity;
    rclcpp::Publisher<joint_msgs::msg::Joints>::SharedPtr ja_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_state;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publish_max_currents;
    rclcpp::Publisher<joint_msgs::msg::NeurlaInput>::SharedPtr publish_with_net;
    Trajectories traj;
    rclcpp::TimerBase::SharedPtr timer_;
    MatrixJoint adder;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    float com_offset;
    float base_lin_vel[3] = { 0.0, 0.0, 0.0};
    float base_ang_vel[3] = { 0.0, 0.0, 0.0};
    float projected_gravity[3] = { 0.0, 0.0, 0.0};
};


#endif
