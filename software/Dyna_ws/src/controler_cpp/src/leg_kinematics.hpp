#ifndef __leg_kinematics__
#define __leg_kinematics__

/*
Logic from https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot
*/

#include <iostream>
#include <cmath>
#include <array>
#include <string>
#include <algorithm>
#include "Eigen/Dense"

#define RIGHT       0
#define LEFT        1

class LegK {
public:
    LegK(uint8_t legtype,
         float shoulder_length,
         float elbow_length,
         float wrist_length);

    float get_domain(float x, float y, float z);
    Eigen::Vector3f solveIK(Eigen::Vector3f xyz_coord);
    Eigen::Vector3f RightIK(float x, float y, float z, float D);
    Eigen::Vector3f LeftIK(float x, float y, float z, float D);
    Eigen::Vector3f solveFK(Eigen::Vector3f joint_angles);
    Eigen::Vector3f RightFK(float shoulder_angle, float elbow_angle, float wrist_angle);
    Eigen::Vector3f LeftFK(float shoulder_angle, float elbow_angle, float wrist_angle);

private:
    uint8_t legtype;
    float shoulder_length;
    float elbow_length;
    float wrist_length;
    std::array<float, 2> hip_lim;
    std::array<float, 2> shoulder_lim;
    std::array<float, 2> leg_lim;
};

#endif