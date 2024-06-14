#include "leg_kinematics.hpp"

LegK::LegK(uint8_t legtype,
         float shoulder_length,
         float elbow_length,
         float wrist_length):
         legtype(legtype),
         shoulder_length(shoulder_length),
         elbow_length(elbow_length),
         wrist_length(wrist_length) {}

float LegK::get_domain(float x, float y, float z){
    float D = (std::pow(y, 2) + std::pow(-z, 2) - std::pow(shoulder_length, 2) +
                std::pow(-x, 2) - std::pow(elbow_length, 2) - std::pow(wrist_length, 2)) / 
                (2 * wrist_length * elbow_length);
    D = std::max(-1.0f, std::min(D, 1.0f));
    return D;
}

Eigen::Vector3f LegK::solveIK(Eigen::Vector3f xyz_coord){
    float x = xyz_coord[0];
    float y = xyz_coord[1];
    float z = xyz_coord[2];

    float D = get_domain(x, y, z);
    if (legtype == RIGHT){
        return RightIK(x, y, z, D);
    } else {
        return LeftIK(x, y, z, D);
    }
}

Eigen::Vector3f LegK::RightIK(float x, float y, float z, float D){
    float wrist_angle = atan2(-sqrt(1 - pow(D,2)), D);
    float sqrt_component = pow(y,2) + pow(z,2) - pow(shoulder_length,2);
    if (sqrt_component < 0.0f){
        sqrt_component = 0.0f;
    }
    float shoulder_angle = -atan2(z, y) - atan2(sqrt(sqrt_component), -shoulder_length);
    float elbow_angle = atan2(-x, sqrt(sqrt_component)) 
                         - atan2(wrist_length * sin(wrist_angle), elbow_length 
                              + wrist_length * cos(wrist_angle));
    Eigen::Vector3f joint_angles = {shoulder_angle, -elbow_angle, -wrist_angle};
    return joint_angles;
}

Eigen::Vector3f LegK::LeftIK(float x, float y, float z, float D){
    float wrist_angle = atan2(-sqrt(1 - pow(D,2)), D);
    float sqrt_component = pow(y,2) + pow(z,2) - pow(shoulder_length,2);
    if (sqrt_component < 0.0f){
        sqrt_component = 0.0f;
    }
    float shoulder_angle = -atan2(z, y) - atan2(sqrt(sqrt_component), shoulder_length);
    float elbow_angle = atan2(-x, sqrt(sqrt_component)) 
                         - atan2(wrist_length * sin(wrist_angle), elbow_length
                            + wrist_length * cos(wrist_angle));
    Eigen::Vector3f joint_angles = {shoulder_angle, -elbow_angle, -wrist_angle};
    return joint_angles;
}

Eigen::Vector3f LegK::solveFK(Eigen::Vector3f joint_angles){
    float shoulder_angle = joint_angles[0]*M_PI/180;
    float elbow_angle = joint_angles[1]*M_PI/180;
    float wrist_angle = joint_angles[2]*M_PI/180;
    if (legtype == RIGHT){
        return RightFK(shoulder_angle, elbow_angle, wrist_angle);
    } else {
        return LeftFK(shoulder_angle, elbow_angle, wrist_angle);
    }            
}

Eigen::Vector3f LegK::RightFK(float shoulder_angle, float elbow_angle, float wrist_angle){
    float x = elbow_length * sin(elbow_angle) + wrist_length * sin(elbow_angle + wrist_angle);
    float y = (-wrist_length * sin(shoulder_angle) * cos(elbow_angle + wrist_angle)
                -shoulder_length * cos(shoulder_angle) 
                -elbow_length * cos(elbow_angle) * sin(shoulder_angle));
    float z = (shoulder_length * sin(shoulder_angle) 
                -elbow_length * cos(shoulder_angle) * cos(elbow_angle)
                -wrist_length * cos(shoulder_angle) * cos(elbow_angle + wrist_angle));
    Eigen::Vector3f xyz_coord = {x, y, z};
    return xyz_coord;
}
    
Eigen::Vector3f LegK::LeftFK(float shoulder_angle, float elbow_angle, float wrist_angle){
    float x = elbow_length * sin(elbow_angle) + wrist_length * sin(elbow_angle + wrist_angle);
    float y = (-wrist_length * sin(shoulder_angle) * cos(elbow_angle + wrist_angle)
                + shoulder_length * cos(shoulder_angle)
                - elbow_length * cos(elbow_angle) * sin(shoulder_angle));
    float z = (-shoulder_length * sin(shoulder_angle)
                -elbow_length * cos(shoulder_angle) * cos(elbow_angle)
                -wrist_length * cos(shoulder_angle) * cos(elbow_angle + wrist_angle));
    Eigen::Vector3f xyz_coord = {x, y, z};
    return xyz_coord;
}
    