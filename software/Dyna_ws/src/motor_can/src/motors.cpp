#include "motors.hpp"

using std::placeholders::_1;

Motors::Motors(): Node("brushless_motors")
{
    declare_leg_config("FR");
    declare_leg_config("FL");
    declare_leg_config("BL");
    declare_leg_config("BR");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Staring Motor Node");
    rclcpp::QoS rmw_qos_profile_sensor_data(24);

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = client_cb_group_;
    rclcpp::SubscriptionOptions options;
    options.callback_group = client_cb_group_;

    subscriber_state= this->create_subscription<std_msgs::msg::Bool>("/motors_state", 10,
        std::bind(&Motors::change_state, this, _1));

    subscriber_joints = this->create_subscription<joint_msgs::msg::Joints>("joint_requests",48,
                                        std::bind(&Motors::request, this, _1));

    subscriber_estimates = this->create_subscription<joint_msgs::msg::JointEstimates>("motor_data",48,
        std::bind(&Motors::data_reception, this, _1));

    publisher_odrive_data = this->create_publisher<joint_msgs::msg::OdriveData>("joint_data",100);

    publisher_request = this->create_publisher<joint_msgs::msg::Joints>("request_positions",24);

    publisher_reboot = this->create_publisher<joint_msgs::msg::JointsBool>("request_reboot",10);

    publisher_axisstate = this->create_publisher<joint_msgs::msg::JointsBool>("request_axisstate",10);

    
    
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Motors::publish_joints, this), timer_cb_group_
    );
    

    std::vector<std::string> positions = {"FR", "FL", "BL", "BR"};
    std::vector<std::string> parts = {"shoulder", "arm", "foot"};

    for (const auto& pos : positions) {
        for (size_t i = 0; i < parts.size(); ++i) {
            std::string motor_name = pos + parts[i];
            brushless_motors.insert(
                std::pair<std::string, BrushlessMotor>(
                    motor_name,
                    BrushlessMotor(
                        static_cast<uint8_t>(this->get_parameter_or(pos + ".motor" + std::to_string(i) + ".motorN", 0)),
                        static_cast<float>(this->get_parameter_or(pos + ".motor" + std::to_string(i) + ".iOffset", 0.0)),
                        static_cast<uint8_t>(this->get_parameter_or(pos + ".motor" + std::to_string(i) + ".axisID", 0)),
                        static_cast<int8_t>(this->get_parameter_or(pos + ".motor" + std::to_string(i) + ".direction", 0)),
                        motor_name,
                        static_cast<float>(this->get_parameter_or(pos + ".motor" + std::to_string(i) + ".joint_max", 0)),
                        static_cast<float>(this->get_parameter_or(pos + ".motor" + std::to_string(i) + ".joint_min", 0))
                    )
                )
            );
        }
    }

    for (auto& pair : brushless_motors) {
        axisID[pair.second.get_axisID()] = pair.first;
    }

    rrate = 9;

    names[0] = "FRshoulder";
    names[1] = "FRarm";
    names[2] = "FRfoot";
    names[3] = "FLshoulder";
    names[4] = "FLarm";
    names[5] = "FLfoot";
    names[6] = "BLshoulder";
    names[7] = "BLarm";
    names[8] = "BLfoot";
    names[9] = "BRshoulder";
    names[10] = "BRarm";
    names[11] = "BRfoot";
}

void Motors::declare_leg_config(const std::string &leg_name)
{
    this->declare_parameter<std::string>(leg_name + ".legType", "");
    for (int i = 0; i < 3; ++i)
    {
        std::string motor_key = leg_name + ".motor" + std::to_string(i);
        this->declare_parameter<int>(motor_key + ".motorN", 0);
        this->declare_parameter<int>(motor_key + ".axisID", 0);
        this->declare_parameter<int>(motor_key + ".direction", 1);
        this->declare_parameter<double>(motor_key + ".iOffset", 0.0);
        this->declare_parameter<double>(motor_key + ".calibration_angle", 0.0);
        this->declare_parameter<double>(motor_key + ".joint_limit_min", 0.0);
        this->declare_parameter<double>(motor_key + ".joint_limit_max", 0.0);
    }
}

void Motors::data_reception(const joint_msgs::msg::JointEstimates::SharedPtr joints)
{   
    // FRshoulder
    brushless_motors.find(axisID[0])->second.update_estimates(joints->frshoulder.position,
                                                              joints->frshoulder.velocity);
    brushless_motors.find(axisID[0])->second.update_current(joints->frshoulder.current);

    // FRarm
    brushless_motors.find(axisID[1])->second.update_estimates(joints->frarm.position,
                                                              joints->frarm.velocity);
    brushless_motors.find(axisID[1])->second.update_current(joints->frarm.current);

    // FRfoot
    brushless_motors.find(axisID[2])->second.update_estimates(joints->frfoot.position,
                                                              joints->frfoot.velocity);
    brushless_motors.find(axisID[2])->second.update_current(joints->frfoot.current);

    // FLshoulder
    brushless_motors.find(axisID[3])->second.update_estimates(joints->flshoulder.position,
                                                              joints->flshoulder.velocity);
    brushless_motors.find(axisID[3])->second.update_current(joints->flshoulder.current);

    // FLarm
    brushless_motors.find(axisID[4])->second.update_estimates(joints->flarm.position,
                                                              joints->flarm.velocity);
    brushless_motors.find(axisID[4])->second.update_current(joints->flarm.current);

    // FLfoot
    brushless_motors.find(axisID[5])->second.update_estimates(joints->flfoot.position,
                                                              joints->flfoot.velocity);
    brushless_motors.find(axisID[5])->second.update_current(joints->flfoot.current);

    // BLshoulder
    brushless_motors.find(axisID[6])->second.update_estimates(joints->blshoulder.position,
                                                              joints->blshoulder.velocity);
    brushless_motors.find(axisID[6])->second.update_current(joints->blshoulder.current);

    // BLarm
    brushless_motors.find(axisID[7])->second.update_estimates(joints->blarm.position,
                                                              joints->blarm.velocity);
    brushless_motors.find(axisID[7])->second.update_current(joints->blarm.current);

    // BLfoot
    brushless_motors.find(axisID[8])->second.update_estimates(joints->blfoot.position,
                                                              joints->blfoot.velocity);
    brushless_motors.find(axisID[8])->second.update_current(joints->blfoot.current);

    // BRshoulder
    brushless_motors.find(axisID[9])->second.update_estimates(joints->brshoulder.position,
                                                              joints->brshoulder.velocity);
    brushless_motors.find(axisID[9])->second.update_current(joints->brshoulder.current);

    // BRarm
    brushless_motors.find(axisID[10])->second.update_estimates(joints->brarm.position,
                                                               joints->brarm.velocity);
    brushless_motors.find(axisID[10])->second.update_current(joints->brarm.current);

    // BRfoot
    brushless_motors.find(axisID[11])->second.update_estimates(joints->brfoot.position,
                                                              joints->brfoot.velocity);
    brushless_motors.find(axisID[11])->second.update_current(joints->brfoot.current);
}


void Motors::change_state(const std_msgs::msg::Bool::SharedPtr msg)
{
    joint_msgs::msg::JointsBool data;
    if (msg->data)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Turning on motors");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Disengaging all motors");
    }

    data.frshoulder = msg->data;
    data.frarm = msg->data;
    data.frfoot = msg->data;
    data.flshoulder = msg->data;
    data.flarm = msg->data;
    data.flfoot = msg->data;
    data.blshoulder = msg->data;
    data.blarm = msg->data;
    data.blfoot = msg->data;
    data.brshoulder = msg->data;
    data.brarm = msg->data;
    data.brfoot = msg->data;
    
    publisher_axisstate->publish(data);
    
}

void Motors::publish_joints()
{
    joint_msgs::msg::OdriveData joints;

    joints.angles.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    joints.angles.frshoulder = brushless_motors.find("FRshoulder")->second.get_angle()/rrate;
    joints.angles.frarm = brushless_motors.find("FRarm")->second.get_angle()/rrate;
    joints.angles.frfoot = brushless_motors.find("FRfoot")->second.get_angle()/rrate;
    joints.angles.flshoulder = brushless_motors.find("FLshoulder")->second.get_angle()/rrate;
    joints.angles.flarm = brushless_motors.find("FLarm")->second.get_angle()/rrate;
    joints.angles.flfoot = brushless_motors.find("FLfoot")->second.get_angle()/rrate;
    joints.angles.blshoulder = brushless_motors.find("BLshoulder")->second.get_angle()/rrate;
    joints.angles.blarm = brushless_motors.find("BLarm")->second.get_angle()/rrate;
    joints.angles.blfoot = brushless_motors.find("BLfoot")->second.get_angle()/rrate;
    joints.angles.brshoulder = brushless_motors.find("BRshoulder")->second.get_angle()/rrate;
    joints.angles.brarm = brushless_motors.find("BRarm")->second.get_angle()/rrate;
    joints.angles.brfoot = brushless_motors.find("BRfoot")->second.get_angle()/rrate;

    joints.angles.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    joints.velocities.frshoulder = brushless_motors.find("FRshoulder")->second.get_velocity_rpm()/rrate;
    joints.velocities.frarm = brushless_motors.find("FRarm")->second.get_velocity_rpm()/rrate;
    joints.velocities.frfoot = brushless_motors.find("FRfoot")->second.get_velocity_rpm()/rrate;
    joints.velocities.flshoulder = brushless_motors.find("FLshoulder")->second.get_velocity_rpm()/rrate;
    joints.velocities.flarm = brushless_motors.find("FLarm")->second.get_velocity_rpm()/rrate;
    joints.velocities.flfoot = brushless_motors.find("FLfoot")->second.get_velocity_rpm()/rrate;
    joints.velocities.blshoulder = brushless_motors.find("BLshoulder")->second.get_velocity_rpm()/rrate;
    joints.velocities.blarm = brushless_motors.find("BLarm")->second.get_velocity_rpm()/rrate;
    joints.velocities.blfoot = brushless_motors.find("BLfoot")->second.get_velocity_rpm()/rrate;
    joints.velocities.brshoulder = brushless_motors.find("BRshoulder")->second.get_velocity_rpm()/rrate;
    joints.velocities.brarm = brushless_motors.find("BRarm")->second.get_velocity_rpm()/rrate;
    joints.velocities.brfoot = brushless_motors.find("BRfoot")->second.get_velocity_rpm()/rrate;

    joints.currents.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    joints.currents.frshoulder = brushless_motors.find("FRshoulder")->second.get_current();
    joints.currents.frarm = brushless_motors.find("FRarm")->second.get_current();
    joints.currents.frfoot = brushless_motors.find("FRfoot")->second.get_current();
    joints.currents.flshoulder = brushless_motors.find("FLshoulder")->second.get_current();
    joints.currents.flarm = brushless_motors.find("FLarm")->second.get_current();
    joints.currents.flfoot = brushless_motors.find("FLfoot")->second.get_current();
    joints.currents.blshoulder = brushless_motors.find("BLshoulder")->second.get_current();
    joints.currents.blarm = brushless_motors.find("BLarm")->second.get_current();
    joints.currents.blfoot = brushless_motors.find("BLfoot")->second.get_current();
    joints.currents.brshoulder = brushless_motors.find("BRshoulder")->second.get_current();
    joints.currents.brarm = brushless_motors.find("BRarm")->second.get_current();
    joints.currents.brfoot = brushless_motors.find("BRfoot")->second.get_current();

    publisher_odrive_data->publish(joints);
}



void Motors::reboot_odrive()
{   
    joint_msgs::msg::JointsBool msg;

    msg.frshoulder = true;
    msg.frarm = true;
    msg.frfoot = true;
    msg.flshoulder = true;
    msg.flarm = true;
    msg.flfoot = true;
    msg.blshoulder = true;
    msg.blarm = true;
    msg.blfoot = true;
    msg.brshoulder = true;
    msg.brarm = true;
    msg.brfoot = true;

    publisher_reboot->publish(msg);
}

void Motors::request(const joint_msgs::msg::Joints::SharedPtr joints)
{   
    float angles[12] = {joints->frshoulder,
                        joints->frarm,
                        joints->frfoot,
                        joints->flshoulder, 
                        joints->flarm,
                        joints->flfoot,
                        joints->blshoulder,
                        joints->blarm,
                        joints->blfoot,
                        joints->brshoulder,
                        joints->brarm,
                        joints->brfoot};

    joint_msgs::msg::Joints msg;
    float posii[12];
    
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        float angle = angles[i];
        if (joints->checkmax){
            if (angle > brushless_motors.find(names[i])->second.get_joint_max())
            {
                angle = brushless_motors.find(names[i])->second.get_joint_max();
                std::cout << "Joint " << names[i] << " is too high" << std::endl;
            } else if (angle < brushless_motors.find(names[i])->second.get_joint_min())
            {
                angle = brushless_motors.find(names[i])->second.get_joint_min();
                std::cout << "Joint " << names[i] << " is too low" << std::endl;
            }
        }
        angle = angle * rrate;
        float position = brushless_motors.find(names[i])->second.get_input_from_angle(angle);

        posii[i] = position;
    } 

    msg.frshoulder = posii[0];
    msg.frarm = posii[1];
    msg.frfoot = posii[2];
    msg.flshoulder = posii[3];
    msg.flarm = posii[4];
    msg.flfoot = posii[5];
    msg.blshoulder = posii[6];
    msg.blarm = posii[7];
    msg.blfoot = posii[8];
    msg.brshoulder = posii[9];
    msg.brarm = posii[10];
    msg.brfoot = posii[11];

    publisher_request->publish(msg);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto moto = std::make_shared<Motors>();
  executor.add_node(moto);
  executor.spin();
//   rclcpp::spin(std::make_shared<Motors>());
  rclcpp::shutdown();
  return 0;
}
