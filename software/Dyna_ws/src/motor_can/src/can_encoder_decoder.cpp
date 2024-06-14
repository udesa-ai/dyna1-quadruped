#include "can_encoder_decoder.hpp"

CanED::CanED() {
    // valtype = {"Pos_Estimate", "Vel_Estimate", "Input_Pos", "Input_Vel", "Input_Torque_FF",
    //            "Input_Torque", "Velocity_Limit", "Current_Limit", "Traj_Vel_Limit",
    //            "Traj_Accel_Limit", "Traj_Decel_Limit", "Traj_Inertia", "Iq_Setpoint",
    //            "Iq_Measured", "Sensorless_Pos_Estimate", "Sensorless_Vel_Estimate", 
    //            "Bus_Voltage", "Bus_Current", "Pos_Gain", "Vel_Gain", "Vel_Integrator_Gain",
    //            "ADC_Voltage"};
}

void CanED::encode_position(float arr, uint8_t* data){
    uint64_t msg_data = 0;
    uint32_t uintValue;
    std::memcpy(&uintValue, &arr, sizeof(arr));
    msg_data |= uintValue;
    for (uint8_t i = 0; i < 8; ++i) {
        data[i] = (msg_data >> (i * 8)) & 0xFF;
    }
}

void CanED::encode_max_current(float maxC, uint8_t* data){
    uint64_t msg_data = 0;
    uint32_t uintValue;
    std::memcpy(&uintValue, &maxC, sizeof(maxC));
    msg_data |= static_cast<uint64_t>(uintValue) << 32;
    for (uint8_t i = 0; i < 8; ++i) {
        data[i] = (msg_data >> (i * 8)) & 0xFF;
    }
}

void CanED::encode_axis_state(uint8_t state, uint8_t* data){
    uint64_t msg_data = 0;
    msg_data |= state;
    for (uint8_t i = 0; i < 8; ++i) {
        data[i] = (msg_data >> (i * 8)) & 0xFF;
    }
}

void CanED::decode_float(const uint8_t* data, float* arr){

    uint64_t msg_data = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        msg_data |= static_cast<uint64_t>(data[i]) << (i * 8);
    }


    float floatValue1, floatValue2;
    uint64_t mask = (1UL << 32) - 1UL;
    uint32_t uintValue1 = static_cast<uint32_t>(msg_data & mask);
    std::memcpy(&floatValue1, &uintValue1, sizeof(uintValue1));   
    arr[0] = floatValue1;

    uint32_t uintValue2 = static_cast<uint32_t>((msg_data >> 32) & mask);
    std::memcpy(&floatValue2, &uintValue2, sizeof(uintValue2));   
    arr[1] = floatValue2;
}

void CanED::getAxisCommand(uint32_t id, uint8_t* axis, uint8_t* command) {
    *command = static_cast<uint8_t>(id & 0x1F);
    *axis = static_cast<uint8_t>((id >> 5) & 0x3F);
}
