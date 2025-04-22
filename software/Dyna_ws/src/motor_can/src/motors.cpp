#include "motors.hpp"

using std::placeholders::_1;

Motors::Motors(): Node("brushless_motors")
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Staring Motor Node");
    this->declare_parameter("fileLocation", "");
    rclcpp::QoS rmw_qos_profile_sensor_data(24);

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = client_cb_group_;
    rclcpp::SubscriptionOptions options;
    options.callback_group = client_cb_group_;

    // publisher_CAN = this->create_publisher<can_msgs::msg::Frame>("CAN/can0/transmit",24);
    // subscriber_CAN = this->create_subscription<can_msgs::msg::Frame>("CAN/can0/receive",
    //                                                    24,
    //                                                    std::bind(&Motors::reception, this, _1),
    //                                                    options);

    subscriber_state= this->create_subscription<std_msgs::msg::Bool>("/motors_state",10,
    std::bind(&Motors::change_state, this, _1));

    subscriber_joints = this->create_subscription<joint_msgs::msg::Joints>("joint_requests",1000,
    std::bind(&Motors::request, this, _1), options);
    
    subscriber_max_current = this->create_subscription<joint_msgs::msg::Joints>("joint_max_currents",1000,
    std::bind(&Motors::change_max_current, this, _1));

    publisher_odrive_data = this->create_publisher<joint_msgs::msg::OdriveData>("joint_data",100);
    
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Motors::publish_joints, this), timer_cb_group_
    );
    
    const std::string jsonFilePath = this->get_parameter("fileLocation").as_string();
    std::ifstream jsonFile(jsonFilePath);  
    if (jsonFile.is_open()) {
        jsonFile >> leg_data;
        jsonFile.close();
    } else {
        std::cout << "Failed to open JSON file" << std::endl;
    }

    canEncDec = CanED();

    // Front Right

    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("FRshoulder", BrushlessMotor(
            static_cast<uint8_t>(leg_data["FR"]["motor0"]["motorN"].asUInt()),
            leg_data["FR"]["motor0"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["FR"]["motor0"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["FR"]["motor0"]["direction"].asInt()),
            "FRshoulder")));
    
    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("FRarm", BrushlessMotor(
            static_cast<uint8_t>(leg_data["FR"]["motor1"]["motorN"].asUInt()),
            leg_data["FR"]["motor1"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["FR"]["motor1"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["FR"]["motor1"]["direction"].asInt()),
            "FRarm")));
    
    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("FRfoot", BrushlessMotor(
            static_cast<uint8_t>(leg_data["FR"]["motor2"]["motorN"].asUInt()),
            leg_data["FR"]["motor2"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["FR"]["motor2"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["FR"]["motor2"]["direction"].asInt()),
            "FRfoot")));

    // Front Left

    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("FLshoulder", BrushlessMotor(
            static_cast<uint8_t>(leg_data["FL"]["motor0"]["motorN"].asUInt()),
            leg_data["FL"]["motor0"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["FL"]["motor0"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["FL"]["motor0"]["direction"].asInt()),
            "FLshoulder")));
    
    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("FLarm", BrushlessMotor(
            static_cast<uint8_t>(leg_data["FL"]["motor1"]["motorN"].asUInt()),
            leg_data["FL"]["motor1"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["FL"]["motor1"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["FL"]["motor1"]["direction"].asInt()),
            "FLarm")));

    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("FLfoot", BrushlessMotor(
            static_cast<uint8_t>(leg_data["FL"]["motor2"]["motorN"].asUInt()),
            leg_data["FL"]["motor2"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["FL"]["motor2"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["FL"]["motor2"]["direction"].asInt()),
            "FLfoot")));

    // Back Left

    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("BLshoulder", BrushlessMotor(
            static_cast<uint8_t>(leg_data["BL"]["motor0"]["motorN"].asUInt()),
            leg_data["BL"]["motor0"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["BL"]["motor0"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["BL"]["motor0"]["direction"].asInt()),
            "BLshoulder")));
    
    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("BLarm", BrushlessMotor(
            static_cast<uint8_t>(leg_data["BL"]["motor1"]["motorN"].asUInt()),
            leg_data["BL"]["motor1"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["BL"]["motor1"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["BL"]["motor1"]["direction"].asInt()),
            "BLarm")));

    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("BLfoot", BrushlessMotor(
            static_cast<uint8_t>(leg_data["BL"]["motor2"]["motorN"].asUInt()),
            leg_data["BL"]["motor2"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["BL"]["motor2"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["BL"]["motor2"]["direction"].asInt()),
            "BLfoot")));

    // Back Right

    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("BRshoulder", BrushlessMotor(
            static_cast<uint8_t>(leg_data["BR"]["motor0"]["motorN"].asUInt()),
            leg_data["BR"]["motor0"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["BR"]["motor0"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["BR"]["motor0"]["direction"].asInt()),
            "BRshoulder")));

    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("BRarm", BrushlessMotor(
            static_cast<uint8_t>(leg_data["BR"]["motor1"]["motorN"].asUInt()),
            leg_data["BR"]["motor1"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["BR"]["motor1"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["BR"]["motor1"]["direction"].asInt()),
            "BRarm")));
    
    brushless_motors.insert(std::pair<std::string, BrushlessMotor>("BRfoot", BrushlessMotor(
            static_cast<uint8_t>(leg_data["BR"]["motor2"]["motorN"].asUInt()),
            leg_data["BR"]["motor2"]["iOffset"].asFloat(),
            static_cast<uint8_t>(leg_data["BR"]["motor2"]["axisID"].asUInt()),
            static_cast<int8_t>(leg_data["BR"]["motor2"]["direction"].asInt()),
            "BRfoot")));

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

void Motors::disengage_all()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Disengaging all motors");
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    canEncDec.encode_axis_state(AXIS_STATE_IDLE, data);
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        uint8_t axis = brushless_motors.find(names[i])->second.get_axisID();
        //publish_CAN(axis, SEX_AXIS_STATE, data, 0);
    }
}

void Motors::engage_all()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Turning on motors");
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    canEncDec.encode_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL, data);
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        uint8_t axis = brushless_motors.find(names[i])->second.get_axisID();
        //publish_CAN(axis, SEX_AXIS_STATE, data, 0);
    }
}

void Motors::change_state(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        engage_all();
    } else {
        disengage_all();
    }
    
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

void Motors::reboot_odrive(std::vector<std::string> selection)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (std::string name : selection) {
        uint8_t axisid = brushless_motors.find(name)->second.get_axisID();
        //publish_CAN(axisid, REBOOT, data, 0);
    }
}

void Motors::request(const joint_msgs::msg::Joints::SharedPtr joints)
{
    float angles[12] = {joints->frshoulder * rrate,
                        joints->frarm * rrate,
                        joints->frfoot * rrate,
                        joints->flshoulder * rrate, 
                        joints->flarm * rrate,
                        joints->flfoot * rrate,
                        joints->blshoulder * rrate,
                        joints->blarm * rrate,
                        joints->blfoot * rrate,
                        joints->brshoulder * rrate,
                        joints->brarm * rrate,
                        joints->brfoot * rrate};
    
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        uint8_t axis = brushless_motors.find(names[i])->second.get_axisID();
        float position = brushless_motors.find(names[i])->second.get_input_from_angle(angles[i]);
        uint8_t data[8];
        canEncDec.encode_position(position, data);
        //publish_CAN(axis, SET_INPUT_POS, data, 0);
    } 
}

void Motors::change_max_current(const joint_msgs::msg::Joints::SharedPtr max_currents)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Changing MAX currents");
    float currents[12] = {max_currents->frshoulder,
                        max_currents->frarm,
                        max_currents->frfoot,
                        max_currents->flshoulder, 
                        max_currents->flarm,
                        max_currents->flfoot,
                        max_currents->blshoulder,
                        max_currents->blarm,
                        max_currents->blfoot,
                        max_currents->brshoulder,
                        max_currents->brarm,
                        max_currents->brfoot};
    
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        uint8_t axis = brushless_motors.find(names[i])->second.get_axisID();
        uint8_t data[8];
        canEncDec.encode_max_current(currents[i], data);
        //publish_CAN(axis, SET_LIMITS, data, 0);
    }    
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
