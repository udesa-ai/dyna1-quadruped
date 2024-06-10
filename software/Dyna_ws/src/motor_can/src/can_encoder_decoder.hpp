#ifndef MOTORS
#define MOTORS

#include <string>
#include <cstdint>
#include <iostream>
#include <cstdarg>
#include <vector>
#include <algorithm>
#include <cstring>

#define HEARTBEAT                   1
#define GET_MOTOR_ERROR             3
#define GET_ENCODER_ERROR           4
#define GET_SENSORLESS_ERROR        5
#define SET_AXIS_NODE_ID            6
#define SEX_AXIS_STATE              7
#define GET_ENCODER_ESTIMATES       9
#define GET_ENCODER_COUNT           10
#define SET_CONTROLLER_MODE         11
#define SET_INPUT_POS               12
#define SET_INPUT_VEL               13
#define SET_INPUT_TORQUE            14
#define SET_LIMITS                  15
#define START_ANTICOGGING           16
#define SET_TRAJ_VEL_LIMIT          17
#define SET_TRAJ_ACCEL_LIMITS       18
#define SET_TRAJ_INERTIA            19
#define GET_IQ                      20
#define GET_SENSORLESS_ESTIMATES    21
#define REBOOT                      22
#define GET_BUS_VOLTAGE_CURRENT     23
#define CLEAR_ERRORS                24
#define SET_LINEAR_COUNT            25
#define SET_POS_GAIN                26
#define SET_VEL_GAINS               27
#define GET_ADC_VOLTAGE             28
#define GET_CONTROLLER_ERROR        29

class CanED {
public:
    CanED();

    void encode_position(float arr, uint8_t* data);

    void encode_axis_state(uint8_t state, uint8_t* data);

    void decode_float(const uint8_t* data, float* arr);

    void getAxisCommand(uint32_t id, uint8_t* axis, uint8_t* command);

    void encode_max_current(float maxC, uint8_t* data);

private:
    // std::vector<std::string> valtype
};


#endif