#include "motors.hpp"

using std::placeholders::_1;

Motors::Motors(): Node("brushless_motors")
{
    // Declare parameters for each leg dynamically
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

    // subscriber_state= this->create_subscription<std_msgs::msg::Bool>("/motors_state",10,
    // std::bind(&Motors::change_state, this, _1));

    // subscriber_joints = this->create_subscription<joint_msgs::msg::Joints>("joint_requests",1000,
    // std::bind(&Motors::request, this, _1), options);

    // subscriber_estimates = this->create_subscription<joint_msgs::msg::Estimates>("encoder_estimates",1000,
    // std::bind(&Motors::encoder_reception, this, _1), options);

    // subscriber_current = this->create_subscription<joint_msgs::msg::Current>("odrive_currents",1000,
    // std::bind(&Motors::current_reception, this, _1), options);
    
    // subscriber_max_current = this->create_subscription<joint_msgs::msg::Joints>("joint_max_currents",1000,
    // std::bind(&Motors::change_max_current, this, _1));

    // publisher_odrive_data = this->create_publisher<joint_msgs::msg::OdriveData>("joint_data",100);

    // publisher_request = this->create_publisher<joint_msgs::msg::CanFloat>("request_joint",24);

    // publisher_maxC = this->create_publisher<joint_msgs::msg::CanFloat>("request_maxc",24);

    // publisher_reboot = this->create_publisher<joint_msgs::msg::CanInt>("request_reboot",24);

    // publisher_axisstate = this->create_publisher<joint_msgs::msg::CanInt>("request_axisstate",24);

    
    
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Motors::publish_joints, this), timer_cb_group_
    );
    

    // Front Right 
    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "FRshoulder", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("FR.motor0.motorN", 0)),
                static_cast<float>(this->get_parameter_or("FR.motor0.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("FR.motor0.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("FR.motor0.direction", 0)),
                "FRshoulder",
                static_cast<float>(this->get_parameter_or("FR.motor0.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("FR.motor0.joint_min", 0))
            )
        )
    );
    
    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "FRarm", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("FR.motor1.motorN", 0)),
                static_cast<float>(this->get_parameter_or("FR.motor1.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("FR.motor1.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("FR.motor1.direction", 0)),
                "FRarm",
                static_cast<float>(this->get_parameter_or("FR.motor1.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("FR.motor1.joint_min", 0))
            )
        )
    );
    
    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "FRfoot", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("FR.motor2.motorN", 0)),
                static_cast<float>(this->get_parameter_or("FR.motor2.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("FR.motor2.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("FR.motor2.direction", 0)),
                "FRfoot",
                static_cast<float>(this->get_parameter_or("FR.motor2.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("FR.motor2.joint_min", 0))
            )
        )
    );


    // Front Left
    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "FLshoulder", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("FL.motor0.motorN", 0)),
                static_cast<float>(this->get_parameter_or("FL.motor0.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("FL.motor0.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("FL.motor0.direction", 0)),
                "FLshoulder",
                static_cast<float>(this->get_parameter_or("FL.motor0.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("FL.motor0.joint_min", 0))
            )
        )
    );

    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "FLarm", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("FL.motor1.motorN", 0)),
                static_cast<float>(this->get_parameter_or("FL.motor1.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("FL.motor1.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("FL.motor1.direction", 0)),
                "FLarm",
                static_cast<float>(this->get_parameter_or("FL.motor1.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("FL.motor1.joint_min", 0))
            )
        )
    );

    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "FLfoot", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("FL.motor2.motorN", 0)),
                static_cast<float>(this->get_parameter_or("FL.motor2.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("FL.motor2.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("FL.motor2.direction", 0)),
                "FLfoot",
                static_cast<float>(this->get_parameter_or("FL.motor2.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("FL.motor2.joint_min", 0))
            )
        )
    );


    // Back Left
    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "BLshoulder", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("BL.motor0.motorN", 0)),
                static_cast<float>(this->get_parameter_or("BL.motor0.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("BL.motor0.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("BL.motor0.direction", 0)),
                "BLshoulder",
                static_cast<float>(this->get_parameter_or("BL.motor0.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("BL.motor0.joint_min", 0))
            )
        )
    );

    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "BLarm", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("BL.motor1.motorN", 0)),
                static_cast<float>(this->get_parameter_or("BL.motor1.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("BL.motor1.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("BL.motor1.direction", 0)),
                "BLarm",
                static_cast<float>(this->get_parameter_or("BL.motor1.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("BL.motor1.joint_min", 0))
            )
        )
    );

    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "BLfoot", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("BL.motor2.motorN", 0)),
                static_cast<float>(this->get_parameter_or("BL.motor2.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("BL.motor2.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("BL.motor2.direction", 0)),
                "BLfoot",
                static_cast<float>(this->get_parameter_or("BL.motor2.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("BL.motor2.joint_min", 0))
            )
        )
    );

    // Back Right
    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "BRshoulder", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("BR.motor0.motorN", 0)),
                static_cast<float>(this->get_parameter_or("BR.motor0.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("BR.motor0.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("BR.motor0.direction", 0)),
                "BRshoulder",
                static_cast<float>(this->get_parameter_or("BR.motor0.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("BR.motor0.joint_min", 0))
            )
        )
    );

    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "BRarm", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("BR.motor1.motorN", 0)),
                static_cast<float>(this->get_parameter_or("BR.motor1.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("BR.motor1.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("BR.motor1.direction", 0)),
                "BRarm",
                static_cast<float>(this->get_parameter_or("BR.motor1.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("BR.motor1.joint_min", 0))
            )
        )
    );

    brushless_motors.insert(
        std::pair<std::string, BrushlessMotor>(
            "BRfoot", 
            BrushlessMotor(
                static_cast<uint8_t>(this->get_parameter_or("BR.motor2.motorN", 0)),
                static_cast<float>(this->get_parameter_or("BR.motor2.iOffset", 0.0)),
                static_cast<uint8_t>(this->get_parameter_or("BR.motor2.axisID", 0)),
                static_cast<int8_t>(this->get_parameter_or("BR.motor2.direction", 0)),
                "BRfoot",
                static_cast<float>(this->get_parameter_or("BR.motor2.joint_max", 0)),
                static_cast<float>(this->get_parameter_or("BR.motor2.joint_min", 0))
            )
        )
    );

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

void Motors::encoder_reception(const joint_msgs::msg::Estimates::SharedPtr joint)
{
    brushless_motors.find(axisID[joint->axis])->second.update_estimates(joint->position, joint->velocity);
}

void Motors::current_reception(const joint_msgs::msg::Current::SharedPtr joint)
{
    brushless_motors.find(axisID[joint->axis])->second.update_current(joint->current);
}

void Motors::state_change(uint8_t state) 
{
    joint_msgs::msg::CanInt msg;
    uint8_t axii[12];
    
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        uint8_t axis = brushless_motors.find(names[i])->second.get_axisID();
        axii[i] = axis;
    }

    msg.data.frshoulder = state;
    msg.axis.frshoulder = axii[0];

    msg.data.frarm = state;
    msg.axis.frarm = axii[1];

    msg.data.frfoot = state;
    msg.axis.frfoot = axii[2];

    msg.data.flshoulder = state;
    msg.axis.flshoulder = axii[3];

    msg.data.flarm = state;
    msg.axis.flarm = axii[4];

    msg.data.flfoot = state;
    msg.axis.flfoot = axii[5];

    msg.data.blshoulder = state;
    msg.axis.blshoulder = axii[6];

    msg.data.blarm = state;
    msg.axis.blarm = axii[7];

    msg.data.blfoot = state;
    msg.axis.blfoot = axii[8];

    msg.data.brshoulder = state;
    msg.axis.brshoulder = axii[9];

    msg.data.brarm = state;
    msg.axis.brarm = axii[10];

    msg.data.brfoot = state;
    msg.axis.brfoot = axii[11];

    publisher_axisstate->publish(msg);
}

void Motors::change_state(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Turning on motors");
        state_change(1);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Disengaging all motors");
        state_change(0);
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



void Motors::reboot_odrive()
{   
    joint_msgs::msg::CanInt msg;
    uint8_t axii[12];
    for (uint8_t i = 0; i < 12 ; i++)
    {
        uint8_t axisid = brushless_motors.find(names[i])->second.get_axisID();
        axii[i] = axisid;
    }

    msg.axis.frshoulder = axii[0];
    msg.axis.frarm = axii[1];
    msg.axis.frfoot = axii[2];
    msg.axis.flshoulder = axii[3];
    msg.axis.flarm = axii[4];
    msg.axis.flfoot = axii[5];
    msg.axis.blshoulder = axii[6];
    msg.axis.blarm = axii[7];
    msg.axis.blfoot = axii[8];
    msg.axis.brshoulder = axii[9];
    msg.axis.brarm = axii[10];
    msg.axis.brfoot = axii[11];

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

    joint_msgs::msg::CanFloat msg;
    float posii[12];
    uint8_t axii[12];
    
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        float angle = angles[i];
        if (joints->check_max){
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
        uint8_t axis = brushless_motors.find(names[i])->second.get_axisID();
        float position = brushless_motors.find(names[i])->second.get_input_from_angle(angle);

        axii[i] = axis;
        posii[i] = position;
    } 

    msg.data.frshoulder = posii[0];
    msg.axis.frshoulder = axii[0];

    msg.data.frarm = posii[1];
    msg.axis.frarm = axii[1];

    msg.data.frfoot = posii[2];
    msg.axis.frfoot = axii[2];

    msg.data.flshoulder = posii[3];
    msg.axis.flshoulder = axii[3];

    msg.data.flarm = posii[4];
    msg.axis.flarm = axii[4];

    msg.data.flfoot = posii[5];
    msg.axis.flfoot = axii[5];

    msg.data.blshoulder = posii[6];
    msg.axis.blshoulder = axii[6];

    msg.data.blarm = posii[7];
    msg.axis.blarm = axii[7];

    msg.data.blfoot = posii[8];
    msg.axis.blfoot = axii[8];

    msg.data.brshoulder = posii[9];
    msg.axis.brshoulder = axii[9];

    msg.data.brarm = posii[10];
    msg.axis.brarm = axii[10];

    msg.data.brfoot = posii[11];
    msg.axis.brfoot = axii[11];

    publisher_request->publish(msg);
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

    joint_msgs::msg::CanFloat msg;
    uint8_t axii[12];
    
    for (uint8_t i = 0; i < 12 ; i++)
    {   
        uint8_t axis = brushless_motors.find(names[i])->second.get_axisID();

        axii[i] = axis;
    }

    msg.data.frshoulder = currents[0];
    msg.axis.frshoulder = axii[0];

    msg.data.frarm = currents[1];
    msg.axis.frarm = axii[1];

    msg.data.frfoot = currents[2];
    msg.axis.frfoot = axii[2];

    msg.data.flshoulder = currents[3];
    msg.axis.flshoulder = axii[3];

    msg.data.flarm = currents[4];
    msg.axis.flarm = axii[4];

    msg.data.flfoot = currents[5];
    msg.axis.flfoot = axii[5];

    msg.data.blshoulder = currents[6];
    msg.axis.blshoulder = axii[6];

    msg.data.blarm = currents[7];
    msg.axis.blarm = axii[7];

    msg.data.blfoot = currents[8];
    msg.axis.blfoot = axii[8];

    msg.data.brshoulder = currents[9];
    msg.axis.brshoulder = axii[9];

    msg.data.brarm = currents[10];
    msg.axis.brarm = axii[10];

    msg.data.brfoot = currents[11];
    msg.axis.brfoot = axii[11];

    publisher_maxC->publish(msg);
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
