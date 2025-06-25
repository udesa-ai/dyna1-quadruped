#include "dyna_real_interface.hpp"

using std::placeholders::_1;

RealInterface::RealInterface(): Node("dyna_real_interface")
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Staring Interface Node");
    /* ############## Parameters ############## */
    this->declare_parameter("MAX_CURRENT",10);
    MAX_CURRENT = (uint8_t) this->get_parameter("MAX_CURRENT").as_int();
    
    /* Controller Params */
    this->declare_parameter("STEPLENGTH_SCALE", 0.0f);
    STEPLENGTH_SCALE = (float) this->get_parameter("STEPLENGTH_SCALE").as_double();

    this->declare_parameter("Z_SCALE_CTRL",0.0f);
    Z_SCALE_CTRL = (float) this->get_parameter("Z_SCALE_CTRL").as_double();

    this->declare_parameter("RPY_SCALE",0.0f);
    RPY_SCALE = (float) this->get_parameter("RPY_SCALE").as_double();
    
    this->declare_parameter("SV_SCALE",0.0f);
    SV_SCALE = (float) this->get_parameter("SV_SCALE").as_double();
    
    this->declare_parameter("CHPD_SCALE",0.0f);
    CHPD_SCALE = (float) this->get_parameter("CHPD_SCALE").as_double();
    
    this->declare_parameter("YAW_SCALE",0.0f);
    YAW_SCALE = (float) this->get_parameter("YAW_SCALE").as_double();

    /* FIXED */
    this->declare_parameter("BaseStepVelocity",0.0f);
    BaseStepVelocity = (float) this->get_parameter("BaseStepVelocity").as_double();
    StepVelocity = BaseStepVelocity;

    /* Stock, use Bumpers to change */
    this->declare_parameter("Tswing",0.0f);
    BaseSwingPeriod = (float) this->get_parameter("Tswing").as_double();
    SwingPeriod = BaseSwingPeriod;

    this->declare_parameter("SwingPeriod_LIMITS", std::vector<float>({0.0f, 1.0f}));
    rclcpp::Parameter param("SwingPeriod_LIMITS", std::vector<float>({}));
    this->get_parameter("SwingPeriod_LIMITS", param);
    std::vector<double> test = param.as_double_array();
    SwingPeriod_LIMITS[0] = (float) test[0];
    SwingPeriod_LIMITS[1] = (float) test[1];

    

    /* Stock, use arrow pads to change */
    this->declare_parameter("BaseClearanceHeight",0.0f);
    BaseClearanceHeight = (float) this->get_parameter("BaseClearanceHeight").as_double();
    ClearanceHeight = BaseClearanceHeight;

    this->declare_parameter("BasePenetrationDepth",0.0f);
    BasePenetrationDepth = (float) this->get_parameter("BasePenetrationDepth").as_double();
    PenetrationDepth = BasePenetrationDepth;

    this->declare_parameter("ClearanceHeight_LIMITS", std::vector<float>({0.0f, 1.0f}));
    auto param2 = this->get_parameter("ClearanceHeight_LIMITS");
    ClearanceHeight_LIMITS[0] = (float)param2.as_double_array()[0];
    ClearanceHeight_LIMITS[1] = (float)param2.as_double_array()[1];

    this->declare_parameter("PenetrationDepth_LIMITS", std::vector<float>({0.0f, 1.0f}));
    auto param3 = this->get_parameter("PenetrationDepth_LIMITS");
    PenetrationDepth_LIMITS[0] = (float)param3.as_double_array()[0];
    PenetrationDepth_LIMITS[1] = (float)param3.as_double_array()[1];

    /* ############## CONTROL VARIABLEs ############## */
    /* joint angles in dictionary */
    joint_angles << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    
    /* joint currents in dictionary */
    joint_currents << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    /* Flag to signal when angles and currents have already been measured atleast once */
    readflag_data = false;
    config_done = false;
    current_set = false;

    mini_cmd.x_velocity = 0.0f;
    mini_cmd.y_velocity = 0.0f;
    mini_cmd.rate = 0.0f;
    mini_cmd.roll = 0.0f;
    mini_cmd.pitch = 0.0f;
    mini_cmd.yaw = 0.0f;
    mini_cmd.z = 0.0f;
    mini_cmd.motion = "Stop";
    mini_cmd.movement = "Stepping";

    /* IMU: R, P, Ax, Ay, Az, Gx, Gy, Gz */
    for (int i = 0; i < 8; i++) {
        imu[i] = 0.0f;
    }

    /* Time */
    time_now = this->get_clock()->now();

    /* Kinematic model of quaduped */
    this->declare_parameter("shoulder_length",0.0f);
    this->declare_parameter("elbow_length",0.0f);
    this->declare_parameter("wrist_length",0.0f);
    this->declare_parameter("hip_x",0.0f);
    this->declare_parameter("hip_y",0.0f);
    this->declare_parameter("foot_x",0.0f);
    this->declare_parameter("foot_y",0.0f);
    this->declare_parameter("height",0.0f);
    this->declare_parameter("com_offset",0.0f);

    com_offset = (float) this->get_parameter("com_offset").as_double();

    quadKine = QuadModel((float) this->get_parameter("shoulder_length").as_double(),
                         (float) this->get_parameter("elbow_length").as_double(),
                         (float) this->get_parameter("wrist_length").as_double(),
                         (float) this->get_parameter("hip_x").as_double(),
                         (float) this->get_parameter("hip_y").as_double(),
                         (float) this->get_parameter("foot_x").as_double(),
                         (float) this->get_parameter("foot_y").as_double(),
                         (float) this->get_parameter("height").as_double(),
                         com_offset);                

    T_bf0 = quadKine.WorldToFoot;
    T_bf = T_bf0;
    T_bh = quadKine.WorldToHip;

    std::map<uint8_t, Eigen::Vector3f>  T_hf;
    T_hf.insert(std::pair<uint8_t, Eigen::Vector3f>(FL, {0.0f ,0.0f, 0.0f}));
    T_hf.insert(std::pair<uint8_t, Eigen::Vector3f>(FR, {0.0f ,0.0f, 0.0f}));
    T_hf.insert(std::pair<uint8_t, Eigen::Vector3f>(BL, {0.0f ,0.0f, 0.0f}));
    T_hf.insert(std::pair<uint8_t, Eigen::Vector3f>(BR, {0.0f ,0.0f, 0.0f}));

    quadKine.HipToFoot(Eigen::Vector3f {0,0,0}, Eigen::Vector3f {com_offset,0,0}, T_bf, &T_hf);

    desired[0] = T_hf[FL];
    desired[1] = T_hf[FR];
    desired[2] = T_hf[BL];
    desired[3] = T_hf[BR];

    this->declare_parameter("dt", 1.0);
    // Phase Lag Per Leg: FL, FR, BL, BR
    std::array<float, 4> dSref = {0.0f, 0.5f, 0.5f, 0.0f};
    std::array<float, 4> dSref_end = {0.0f, 0.0f, 0.0f, 0.0f};
    // std::array<float, 4> dSref = {0.0f, 0.5f, 0.5f, 0.0f};
    bzg = BezierGait(dSref, dSref_end, (float) this->get_parameter("dt").as_double(), BaseSwingPeriod, STEPLENGTH_SCALE);

    
    /* ############## SUBSCRIBERS ############## */

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = client_cb_group_;
    rclcpp::SubscriptionOptions options;
    options.callback_group = client_cb_group_;

    sub_cmd = this->create_subscription<joint_msgs::msg::MiniCmd>("mini_cmd",1,
    std::bind(&RealInterface::cmd_cb, this, _1));

    sub_jb = this->create_subscription<teleop_msgs::msg::JoyButtons>("joybuttons",1,
    std::bind(&RealInterface::jb_cb, this, _1));


    jbreleased = true;
    sbreleased = true;
    start_movement = false;
    standing = true;
    motor_states = 0;
    uptime = 3;
    stood = false;
    descend = false;

    /* Subscription in charge of receiving new data */
    subscription_joint_data = this->create_subscription<joint_msgs::msg::OdriveData>("joint_data",
    10,std::bind(&RealInterface::update_data, this, _1));

    errors_data = this->create_subscription<error_msgs::msg::Error>("error_state",
    10,std::bind(&RealInterface::error_update, this, _1));

    sub_imu = this->create_subscription<custom_sensor_msgs::msg::IMUdata>("IMU",
    10,std::bind(&RealInterface::imu_cb, this, _1));

    /* ############## PUBLISHERS ############## */
    ja_pub = this->create_publisher<joint_msgs::msg::Joints>("joint_requests",1);
    motor_state = this->create_publisher<std_msgs::msg::Bool>("motors_state",1);

    publish_max_currents = this->create_publisher<std_msgs::msg::Float32>("request_maxc",10);

    traj = Trajectories(POLYNOMIAL);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RealInterface::control, this), timer_cb_group_
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"READY TO GO!");
}

void RealInterface::imu_cb(const custom_sensor_msgs::msg::IMUdata::SharedPtr data)
{
    imu[0] = data->roll;
    imu[1] = data->pitch;
    imu[2] = (data->gyro_x) * (M_PI / 180);
    imu[3] = (data->gyro_y) * (M_PI / 180);
    imu[4] = (data->gyro_z) * (M_PI / 180);
    imu[5] = data->acc_x;
    imu[6] = data->acc_y;
    imu[7] = (data->acc_z) - 9.81;
}

void RealInterface::error_update(error_msgs::msg::Error::SharedPtr data)
{
    if (data->motor_error != 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Motor Error Detected, Shuting down");
        ERROR_STATE = 1;
        std_msgs::msg::Bool msg;
        msg.data = false;
        motor_states = 0;
        motor_state->publish(msg);
    }

}

void RealInterface::update_data(joint_msgs::msg::OdriveData::SharedPtr data)
{
    joint_angles << data->angles.flshoulder, data->angles.flarm, data->angles.flfoot,
                    data->angles.frshoulder, data->angles.frarm, data->angles.frfoot,
                    data->angles.blshoulder, data->angles.blarm, data->angles.blfoot,
                    data->angles.brshoulder, data->angles.brarm, data->angles.brfoot;
    
    joint_velocities_rpm << data->velocities.flshoulder, data->velocities.flarm, data->velocities.flfoot,
                      data->velocities.frshoulder, data->velocities.frarm, data->velocities.frfoot,
                      data->velocities.blshoulder, data->velocities.blarm, data->velocities.blfoot,
                      data->velocities.brshoulder, data->velocities.brarm, data->velocities.brfoot;

    joint_currents << data->currents.flshoulder, data->currents.flarm, data->currents.flfoot,
                      data->currents.frshoulder, data->currents.frarm, data->currents.frfoot,
                      data->currents.blshoulder, data->currents.blarm, data->currents.blfoot,
                      data->currents.brshoulder, data->currents.brarm, data->currents.brfoot;

    if (!readflag_data){
        readflag_data = true;
    }
}

void RealInterface::cmd_cb(joint_msgs::msg::MiniCmd::SharedPtr data)
{
    mini_cmd.motion = data->motion;
    mini_cmd.movement = data->movement;
    mini_cmd.x_velocity = data->x_velocity;
    mini_cmd.y_velocity = data->y_velocity;
    mini_cmd.rate = data->rate;
    mini_cmd.roll = data->roll;
    mini_cmd.pitch = data->pitch;
    mini_cmd.yaw = data->yaw;
    mini_cmd.z = data->z;
    mini_cmd.faster = data->faster;
    mini_cmd.slower = data->slower;
}

void RealInterface::jb_cb(teleop_msgs::msg::JoyButtons::SharedPtr data)
{
    jb.updown = data->updown;
    jb.leftright = data->leftright;
    jb.left_bump = data->left_bump;
    jb.right_bump = data->right_bump;
    jb.start_b = data->start_b;

    if (ERROR_STATE == 0)
    {
        if (data->left_bump && jbreleased){
            std_msgs::msg::Bool msg;
            if (motor_states == 0){
                msg.data = true;
                motor_states = 1;
            } else {
                msg.data = false;
                motor_states = 0;
            }
            motor_state->publish(msg);
            jbreleased = false;
        }
        if (!data->left_bump){
            jbreleased = true;
        }
        
        if (data->start_b && sbreleased && !start_movement)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Movement started!");
            start_movement = true;
            sbreleased = false;
            MatrixJoint xyz = get_xyz();
            stood = true;

            uint8_t index = 0;
            for (auto& pair : T_bh) {
                Eigen::Matrix4f foot = pair.second;
                
                foot.block(0,3,3,1) += xyz.block(index,0,1,3).transpose();
                pair.second = foot;
                adder.block(index,0,1,3) = desired[index].transpose() - xyz.block(index,0,1,3);
                index++;
            }

            upt0 = this->get_clock()->now();
        } else if (data->start_b && sbreleased && !standing && stood)
        {
            descend = true;
            upt0 = this->get_clock()->now();
        }

        if (!data->start_b) {
            sbreleased = true;
        }
    }
    
}

void RealInterface::set_current(uint8_t max_current){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),(std::string("Configuring currents to ") + std::to_string(max_current)).c_str());
    std_msgs::msg::Float32 msg;
    // convert to float
    msg.data = (float) max_current;
    publish_max_currents->publish(msg);
}
    
void RealInterface::control(){
    if (ERROR_STATE == 0)
    {
        if (!config_done) {
            if (!current_set) {
                set_current(MAX_CURRENT);
                current_set = true;
            }
            if (readflag_data && current_set) {
                config_done = true;
            }
        } else if (start_movement) {
            rclcpp::Time tnow;
            if (standing){
                tnow = this->get_clock()->now();
                rclcpp::Duration duration = tnow - upt0;
                if (duration.seconds() > uptime) {
                    standing = false;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Done Standing");
                } else {
                    rclcpp::Duration tmove = tnow - upt0;
                    TransfDict T_bh_command = T_bh;
                    uint8_t index = 0;
                    for (auto& pair : T_bh_command) {
                        Eigen::Matrix4f foot = pair.second;
                        foot.block(0,3,3,1) += adder.block(index++,0,1,3).transpose()*traj.get_value(tmove.seconds()/uptime);
                        pair.second = foot;
                    }

                    MatrixJoint ja = quadKine.IK({0,0,0}, {-com_offset,0,0}, T_bh_command);

                    publishall(ja);
                }
            } else if (descend) {
                tnow = this->get_clock()->now();
                rclcpp::Duration duration = tnow - upt0;
                if (duration.seconds() > uptime) {
                    std_msgs::msg::Bool msg;
                    msg.data = false;
                    motor_states = 0;
                    motor_state->publish(msg);
                    start_movement = false;
                } else {
                    TransfDict T_bh_command = T_bh;
                    uint8_t index = 0;
                    for (auto& pair : T_bh_command) {
                        Eigen::Matrix4f foot = pair.second;
                        
                        foot.block(0,3,3,1) += adder.block(index++,0,1,3).transpose()*(1-traj.get_value(duration.seconds()/uptime));
                        pair.second = foot;
                    }
                    MatrixJoint ja = quadKine.IK({0,0,0}, {-com_offset,0,0}, T_bh_command);
                    publishall(ja);
                }
            } else {
                move();
            }
        }
    }
    
    
}

MatrixJoint RealInterface::get_xyz()
{
    MatrixJoint xyz = quadKine.FK(joint_angles);
    return xyz;
}
    
void RealInterface::move(){
    Eigen::Vector3f pos = {0.0f, 0.0f ,0.0f};
    Eigen::Vector3f orn = {0.0f ,0.0f ,0.0f};
    float StepLength = 0.0f;
    float LateralFraction = 0.0f;
    float YawRate = 0.0f;


    if (mini_cmd.motion != "Stop")
    {
        StepVelocity = BaseStepVelocity;
        SwingPeriod = std::max(
                        std::min(BaseSwingPeriod + (-mini_cmd.faster - mini_cmd.slower) * SV_SCALE,
                                 SwingPeriod_LIMITS[1]),
                        SwingPeriod_LIMITS[0]);
        

        if (mini_cmd.movement == "Stepping") {
            StepLength = mini_cmd.x_velocity + std::fabs(mini_cmd.y_velocity * 0.66f);
            StepLength = std::max( std::min(StepLength,1.0f), -1.0f);
            StepLength *= STEPLENGTH_SCALE;
            LateralFraction = mini_cmd.y_velocity * M_PI / 2;
            YawRate = mini_cmd.rate * YAW_SCALE;
            pos << 0.0f, 0.0f, 0.0f;
            orn << 0.0f, 0.0f, 0.0f;

        } else {
            StepLength = 0.0f;
            LateralFraction = 0.0f;
            YawRate = 0.0f;

            ClearanceHeight = BaseClearanceHeight;
            PenetrationDepth = BasePenetrationDepth;
            StepVelocity = BaseStepVelocity;

            pos << 0.0f, 0.0f, mini_cmd.z * Z_SCALE_CTRL;
            orn << mini_cmd.roll * RPY_SCALE, mini_cmd.pitch * RPY_SCALE, mini_cmd.yaw * RPY_SCALE;
        }
    } else {
        StepLength = 0.0f;
        LateralFraction = 0.0f;
        YawRate = 0.0f;

        ClearanceHeight = BaseClearanceHeight;
        PenetrationDepth = BasePenetrationDepth;
        StepVelocity = BaseStepVelocity;
        SwingPeriod = BaseSwingPeriod;
        pos << 0.0f, 0.0f, 0.0f;
        orn << 0.0f, 0.0f, 0.0f;
    }
    
    ClearanceHeight += jb.updown * CHPD_SCALE;
    PenetrationDepth += jb.leftright * CHPD_SCALE;

    if (jb.right_bump){
        ClearanceHeight = BaseClearanceHeight;
        PenetrationDepth = BasePenetrationDepth;
        StepVelocity = BaseStepVelocity;
        SwingPeriod = BaseSwingPeriod;
    }
    
    rclcpp::Time tnow;
    tnow = this->get_clock()->now();
    rclcpp::Duration dt = tnow - time_now;

    time_now = tnow;

    bzg.Tswing_ref = SwingPeriod;

    ClearanceHeight = std::max(
                            std::min(ClearanceHeight, ClearanceHeight_LIMITS[1]),
                            ClearanceHeight_LIMITS[0]);

    PenetrationDepth = std::max(
                            std::min(PenetrationDepth, PenetrationDepth_LIMITS[1]),
                            PenetrationDepth_LIMITS[0]);

    
    T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction,
                                            YawRate, StepVelocity,
                                            T_bf0,
                                            ClearanceHeight,
                                            PenetrationDepth,
                                            dt.seconds());

    TransfDict T_bf_copy = T_bf;

    MatrixJoint angles_now = quadKine.IK(orn, pos, T_bf_copy);

    publishall(angles_now);
}

void RealInterface::publishall(MatrixJoint angles){
    joint_msgs::msg::Joints ja_msg;
    
    /*
    ja_msg.header.stamp.sec = int(np.floor(t))
    ja_msg.header.stamp.nanosec = int((t - np.floor(t))*1000000000)
    */

    ja_msg.flshoulder = angles(0, 0) * 180/M_PI;
    ja_msg.flarm = angles(0, 1) * 180/M_PI;
    ja_msg.flfoot = angles(0, 2) * 180/M_PI;

    ja_msg.frshoulder = angles(1, 0) * 180/M_PI;
    ja_msg.frarm = angles(1, 1) * 180/M_PI;
    ja_msg.frfoot = angles(1, 2) * 180/M_PI;

    ja_msg.blshoulder = angles(2, 0) * 180/M_PI;
    ja_msg.blarm = angles(2, 1) * 180/M_PI;
    ja_msg.blfoot = angles(2, 2) * 180/M_PI;

    ja_msg.brshoulder = angles(3, 0) * 180/M_PI;
    ja_msg.brarm = angles(3, 1) * 180/M_PI;
    ja_msg.brfoot = angles(3, 2) * 180/M_PI;

    ja_msg.motion = mini_cmd.motion;
    ja_msg.movement = mini_cmd.movement;

    ja_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    ja_pub->publish(ja_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealInterface>());
  rclcpp::shutdown();
  return 0;
}
