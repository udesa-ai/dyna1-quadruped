#ifndef __algebra__
#define __algebra__

/*
NOTE: Code snippets from Modern Robotics at Northwestern University:
See https://github.com/NxRLab/ModernRobotics
*/

#include "Eigen/Dense"
#include <cmath>

typedef Eigen::Matrix<float, 6, 6> Matrix6f;


Eigen::Matrix4f RpToTrans(Eigen::Matrix3f R, Eigen::Vector3f p);

void TransToRp(Eigen::Matrix4f T, Eigen::Matrix3f *R, Eigen::Vector3f *p);

Eigen::Matrix4f TransInv(Eigen::Matrix4f T);

Eigen::Matrix3f VecTos03(Eigen::Vector3f omg);

Matrix6f Adjoint(Eigen::Matrix4f T);

Eigen::Matrix4f RPY(float roll, float pitch, float yaw);

Eigen::Matrix4f RotateTranslate(Eigen::Matrix4f rotation, Eigen::Vector3f position);

Eigen::Vector3f TransformVector(Eigen::Vector3f xyz_coord, Eigen::Matrix4f rotation, Eigen::Vector3f translation);

#endif