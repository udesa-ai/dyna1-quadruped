#ifndef __brushless__
#define __brushless__

#include <iostream>
#include <string>
#include <map>
#include <cstdint>

#define GPIO_MODE_DIGITAL           0
#define GPIO_MODE_DIGITAL_PULL_UP   1
#define GPIO_MODE_DIGITAL_PULL_DOWN 2
#define GPIO_MODE_ANALOG_IN         3
#define GPIO_MODE_UART_A            4
#define GPIO_MODE_UART_B            5
#define GPIO_MODE_UART_C            6
#define GPIO_MODE_CAN_A             7
#define GPIO_MODE_I2C_A             8
#define GPIO_MODE_SPI_A             9
#define GPIO_MODE_PWM               10
#define GPIO_MODE_ENC0              11
#define PIO_MODE_ENC1               12
#define GPIO_MODE_ENC2              13
#define GPIO_MODE_MECH_BRAKE        14
#define GPIO_MODE_STATUS            15


#define AXIS_STATE_UNDEFINED                            0
#define AXIS_STATE_IDLE                                 1
#define AXIS_STATE_STARTUP_SEQUENCE                     2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE            3
#define AXIS_STATE_MOTOR_CALIBRATION                    4
#define AXIS_STATE_ENCODER_INDEX_SEARCH                 6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION           7
#define AXIS_STATE_CLOSED_LOOP_CONTROL                  8
#define AXIS_STATE_LOCKIN_SPIN                          9
#define AXIS_STATE_ENCODER_DIR_FIND                     10
#define AXIS_STATE_HOMING                               11
#define AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION    12
#define AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION       13


#define ENCODER_MODE_INCREMENTAL        0
#define ENCODER_MODE_HALL               1
#define ENCODER_MODE_SINCOS             2
#define ENCODER_MODE_SPI_ABS_CUI        256
#define ENCODER_MODE_SPI_ABS_AMS        257
#define ENCODER_MODE_SPI_ABS_AEAT       258
#define ENCODER_MODE_SPI_ABS_RLS        259
#define ENCODER_MODE_SPI_ABS_MA732      260


#define CONTROL_MODE_VOLTAGE_CONTROL    0
#define CONTROL_MODE_TORQUE_CONTROL     1
#define CONTROL_MODE_VELOCITY_CONTROL   2
#define CONTROL_MODE_POSITION_CONTROL   3


#define INPUT_MODE_INACTIVE         0
#define INPUT_MODE_PASSTHROUGH      1
#define INPUT_MODE_VEL_RAMP         2
#define INPUT_MODE_POS_FILTER       3
#define INPUT_MODE_MIX_CHANNELS     4
#define INPUT_MODE_TRAP_TRAJ        5
#define INPUT_MODE_TORQUE_RAMP      6
#define INPUT_MODE_MIRROR           7
#define INPUT_MODE_TUNING           8


#define ODRIVE_ERROR_NONE                        0x00
#define ODRIVE_ERROR_CONTROL_ITERATION_MISSED    0x01
#define ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE        0x02
#define ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE         0x04
#define ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT   0x08
#define ODRIVE_ERROR_DC_BUS_OVER_CURRENT         0x10
#define ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION    0x20
#define ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN        0x40
#define ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE    0x80


#define CAN_ERROR_NONE                           0x00
#define CAN_ERROR_DUPLICATE_CAN_IDS              0x01


#define AXIS_ERROR_NONE                          0x00000
#define AXIS_ERROR_INVALID_STATE                 0x00001
#define AXIS_ERROR_MOTOR_FAILED                  0x00040
#define AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED   0x00080
#define AXIS_ERROR_ENCODER_FAILED                0x00100
#define AXIS_ERROR_CONTROLLER_FAILED             0x00200
#define AXIS_ERROR_WATCHDOG_TIMER_EXPIRED        0x00800
#define AXIS_ERROR_MIN_ENDSTOP_PRESSED           0x01000
#define AXIS_ERROR_MAX_ENDSTOP_PRESSED           0x02000
#define AXIS_ERROR_ESTOP_REQUESTED               0x04000
#define AXIS_ERROR_HOMING_WITHOUT_ENDSTOP        0x20000
#define AXIS_ERROR_OVER_TEMP                     0x40000
#define AXIS_ERROR_UNKNOWN_POSITION              0x80000


#define MOTOR_ERROR_NONE                            0x00000000
#define MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE   0x00000001
#define MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE   0x00000002
#define MOTOR_ERROR_DRV_FAULT                       0x00000008
#define MOTOR_ERROR_CONTROL_DEADLINE_MISSED         0x00000010
#define MOTOR_ERROR_MODULATION_MAGNITUDE            0x00000080
#define MOTOR_ERROR_CURRENT_SENSE_SATURATION        0x00000400
#define MOTOR_ERROR_CURRENT_LIMIT_VIOLATION         0x00001000
#define MOTOR_ERROR_MODULATION_IS_NAN               0x00010000
#define MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP      0x00020000
#define MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP        0x00040000
#define MOTOR_ERROR_TIMER_UPDATE_MISSED             0x00080000
#define MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE 0x00100000
#define MOTOR_ERROR_CONTROLLER_FAILED               0x00200000
#define MOTOR_ERROR_I_BUS_OUT_OF_RANGE              0x00400000
#define MOTOR_ERROR_BRAKE_RESISTOR_DISARMED         0x00800000
#define MOTOR_ERROR_SYSTEM_LEVEL                    0x01000000
#define MOTOR_ERROR_BAD_TIMING                      0x02000000
#define MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE          0x04000000
#define MOTOR_ERROR_UNKNOWN_PHASE_VEL               0x08000000
#define MOTOR_ERROR_UNKNOWN_TORQUE                  0x10000000
#define MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND         0x20000000
#define MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT     0x40000000
#define MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE            0x80000000
#define MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND         0x100000000
#define MOTOR_ERROR_UNKNOWN_GAINS                   0x200000000
#define MOTOR_ERROR_CONTROLLER_INITIALIZING         0x400000000
#define MOTOR_ERROR_UNBALANCED_PHASES               0x800000000


#define CONTROLLER_ERROR_NONE                    0x00
#define CONTROLLER_ERROR_OVERSPEED               0x01
#define CONTROLLER_ERROR_INVALID_INPUT_MODE      0x02
#define CONTROLLER_ERROR_UNSTABLE_GAIN           0x04
#define CONTROLLER_ERROR_INVALID_MIRROR_AXIS     0x08
#define CONTROLLER_ERROR_INVALID_LOAD_ENCODER    0x10
#define CONTROLLER_ERROR_INVALID_ESTIMATE        0x20
#define CONTROLLER_ERROR_INVALID_CIRCULAR_RANGE  0x40
#define CONTROLLER_ERROR_SPINOUT_DETECTED        0x80


#define ENCODER_ERROR_NONE                       0x000
#define ENCODER_ERROR_UNSTABLE_GAIN              0x001
#define ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH     0x002
#define ENCODER_ERROR_NO_RESPONSE                0x004
#define ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE   0x008
#define ENCODER_ERROR_ILLEGAL_HALL_STATE         0x010
#define ENCODER_ERROR_INDEX_NOT_FOUND_YET        0x020
#define ENCODER_ERROR_ABS_SPI_TIMEOUT            0x040
#define ENCODER_ERROR_ABS_SPI_COM_FAIL           0x080
#define ENCODER_ERROR_ABS_SPI_NOT_READY          0x100
#define ENCODER_ERROR_HALL_NOT_CALIBRATED_YET    0x200


class BrushlessMotor {
public:
    BrushlessMotor(uint8_t motorn, float pos0, uint8_t axisID, int8_t direc, const std::string name);

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
};


#endif

