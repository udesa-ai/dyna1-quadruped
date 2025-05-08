#ifndef __brushless__
#define __brushless__

#include <iostream>
#include <string>
#include <map>
#include <cstdint>




class BrushlessMotor {
public:
    BrushlessMotor(uint8_t motorn, float pos0, uint8_t axisID, int8_t direc, const std::string& name, float joint_max, float joint_min);

    uint8_t get_axisID();
    void set_pos0(float value);
    float get_position();
    float ang_to_pos(float angle);
    float pos_to_angle(float position);
    float get_angle();
    float get_input_from_pos(float position);
    float get_input_from_angle(float angle);
    float get_current();
    float get_input_from_torque(float torque);
    float get_torque();
    float get_velocity_rpm();
    void update_estimates(float position, float velocity);
    void update_current(float current);
    ~BrushlessMotor();
    float get_joint_max();
    float get_joint_min();

private:
    std::string name;
    uint8_t mnumber;
    float pos0;
    int8_t direc;
    uint8_t axisId;
    float pos_estimate;
    float vel_estimate;
    float Iq_measured;
    float config_torque_constant;
    float joint_max;
    float joint_min;
};


#endif

