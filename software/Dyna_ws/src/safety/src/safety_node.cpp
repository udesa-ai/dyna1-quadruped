#include "safety_node.hpp"

using std::placeholders::_1;

SafetyPrecautions::SafetyPrecautions(): Node("safety_node")
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Staring Safety Node");
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = client_cb_group_;
    rclcpp::SubscriptionOptions options;
    options.callback_group = client_cb_group_;

    /* Subscription in charge of receiving new data */
    subscription_joint_data = this->create_subscription<joint_msgs::msg::OdriveData>("joint_data",
    10,std::bind(&SafetyPrecautions::angles_currents, this, _1), options);

    error_state = this->create_publisher<error_msgs::msg::Error>("error_state",1);

    alpha = 0.01;
    max_acceptable_current = 20;
}

void SafetyPrecautions::angles_currents(joint_msgs::msg::OdriveData::SharedPtr data)
{
    // joint_angles[0] = data->angles.flshoulder;
    // joint_angles[1] = data->angles.flarm;
    // joint_angles[2] = data->angles.flfoot;
    // joint_angles[3] = data->angles.frshoulder;
    // joint_angles[4] = data->angles.frarm;
    // joint_angles[5] = data->angles.frfoot;
    // joint_angles[6] = data->angles.blshoulder;
    // joint_angles[7] = data->angles.blarm;
    // joint_angles[8] = data->angles.blfoot;
    // joint_angles[9] = data->angles.brshoulder;
    // joint_angles[10] = data->angles.brarm;
    // joint_angles[11] = data->angles.brfoot;

    joint_currents[0] = data->currents.flshoulder;
    joint_currents[1] = data->currents.flarm;
    joint_currents[2] = data->currents.flfoot;
    joint_currents[3] = data->currents.frshoulder;
    joint_currents[4] = data->currents.frarm;
    joint_currents[5] = data->currents.frfoot;
    joint_currents[6] = data->currents.blshoulder;
    joint_currents[7] = data->currents.blarm;
    joint_currents[8] = data->currents.blfoot;
    joint_currents[9] = data->currents.brshoulder;
    joint_currents[10] = data->currents.brarm;
    joint_currents[11] = data->currents.brfoot;

    error_msgs::msg::Error msg;
    msg.encoder_error = 0;
    msg.motor_error = 0;

    // for(uint8_t i = 0; i < 12; i++){
    //     if (joint_angles[i] == joint_angles_previous[i]){
    //         encoder_check[i]++;
    //     }
    //     if (encoder_check[i] > 5){
    //         msg.encoder_error |= 1 << i;
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),(std::string("Encoder error motor number ") + std::to_string(i+1)).c_str());
    //     }

    //     joint_angles_previous[i] = joint_angles[i];
    // }
    for(uint8_t i = 0; i < 12; i++){
        current_check[i] = joint_currents[i] * alpha + (1-alpha) * current_check[i];
        if (current_check[i] > max_acceptable_current){
            msg.motor_error |= 1 << i;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),(std::string("Current overpass motor number ") + std::to_string(i+1)).c_str());
        }
    }

    error_state->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyPrecautions>());
  rclcpp::shutdown();
  return 0;
}
