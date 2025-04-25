#include "brushless.hpp"

BrushlessMotor::BrushlessMotor(uint8_t motorn, float pos0, uint8_t axisID, int8_t direc, const std::string name, float joint_max, float joint_min)
    : name(name), mnumber(motorn), pos0(pos0), direc(direc), axisId(axisID), joint_max(joint_max), joint_min(joint_min)
    {
        pos_estimate = 0;
        vel_estimate = 0;
        Iq_measured = 0;
        config_torque_constant = 0.0344088;
    }

uint8_t BrushlessMotor::get_axisID() {
    return axisId;
}

void BrushlessMotor::set_pos0(float value) {
    pos0 = value;
}

float BrushlessMotor::get_position() {
    return  direc * pos_estimate;
}

float BrushlessMotor::ang_to_pos(float angle) {
    float position = angle/360 + pos0;
    return position;
}
float BrushlessMotor::pos_to_angle(float position) {
    float angle = (position - pos0)*360;
    return angle;
}

float BrushlessMotor::get_angle() {
    float position = get_position();
    float angle = pos_to_angle(position);
    return angle;
}

float BrushlessMotor::get_input_from_pos(float position) {
    return direc * position;
}

float BrushlessMotor::get_input_from_angle(float angle) {
    float position = ang_to_pos(angle);
    float new_position = get_input_from_pos(position);
    return new_position;
}

float BrushlessMotor::get_current() {
    return direc * Iq_measured;
}

float BrushlessMotor::get_input_from_torque(float torque) {
    return direc * torque;
}

float BrushlessMotor::get_torque() {
    float current = get_current();
    return config_torque_constant * current;
}

float BrushlessMotor::get_velocity_rpm() {
    return direc * 60 * vel_estimate;
}

void BrushlessMotor::update_estimates(float position, float velocity) {
    pos_estimate = position;
    vel_estimate = velocity;
}

void BrushlessMotor::update_current(float current) {
    Iq_measured = current;
}

float BrushlessMotor::get_joint_max() {
    return (float) joint_max;
}

float BrushlessMotor::get_joint_min() {
    return (float) joint_min;
}

BrushlessMotor::~BrushlessMotor(){}