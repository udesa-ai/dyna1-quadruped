#ifndef __quad_kinematics__
#define __quad_kinematics__


#include "leg_kinematics.hpp"
#include "algebra.hpp"
#include <map>
#include <iostream>


#define FL      0
#define FR      1
#define BL      2
#define BR      3

typedef Eigen::Matrix<float, 4, 3> MatrixJoint;


typedef std::map<uint8_t, Eigen::Matrix4f> TransfDict;

class QuadModel {
public:
    QuadModel();
    
    QuadModel(float shoulder_length,
              float elbow_length,
              float wrist_length,
              float hip_x, 
              float hip_y, 
              float foot_x, 
              float foot_y,
              float height,
              float com_offset);

    void HipToFoot(Eigen::Vector3f orn, Eigen::Vector3f pos,
                   TransfDict T_bf,
                   std::map<uint8_t, Eigen::Vector3f> *HipToFoot_List);
    
    MatrixJoint IK(Eigen::Vector3f orn, Eigen::Vector3f pos,
                         TransfDict T_bf);

    MatrixJoint FK(MatrixJoint joint_angles);
    
    std::array<uint8_t, 4> names;

    TransfDict WorldToFoot;
    TransfDict WorldToHip;
    
private:
    float shoulder_length;
    float elbow_length;
    float wrist_length;
    float hip_x;
    float hip_y;
    float foot_x; 
    float foot_y;
    float height;
    float com_offset;
    std::array<float, 2> shoulder_lim;
    std::array<float, 2> elbow_lim;
    std::array<float, 2> wrist_lim;
    std::map<uint8_t, LegK> Legs;
    
};

#endif