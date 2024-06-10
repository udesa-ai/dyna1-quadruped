#include "algebra.hpp"
#include "iostream"

Eigen::Matrix4f RpToTrans(Eigen::Matrix3f R, Eigen::Vector3f p){
    Eigen::Matrix4f T = Eigen::Matrix4f::Zero(4,4);
    T.block(0,0,3,3) << R;
    T.block(0,3,3,1) << p;
    T(3,3) = 1.0f;
    return T;
}

void TransToRp(Eigen::Matrix4f T, Eigen::Matrix3f *R, Eigen::Vector3f *p){
    *R = T.block(0,0,3,3);

    *p = T.block(0,3,3,1);
}

Eigen::Matrix4f TransInv(Eigen::Matrix4f T){
    return T.inverse();
}

Eigen::Matrix3f VecTos03(Eigen::Vector3f omg){
    Eigen::Matrix3f answer {{0.0f, -omg[2], omg[1]},
                            {omg[2], 0.0f, -omg[0]},
                            {-omg[1], omg[0], 0.0f}};
    return answer;
}

Matrix6f Adjoint(Eigen::Matrix4f T){
    Eigen::Matrix3f R;
    Eigen::Vector3f p;
    TransToRp(T, &R, &p);
    Matrix6f A = Matrix6f::Zero(6,6);
    A.block(0,0,3,3) << R;
    A.block(3,3,3,3) << R;
    A.block(3,0,3,3) << VecTos03(p)*R;
    return A;
}


Eigen::Matrix4f RPY(float roll, float pitch, float yaw){
    Eigen::Matrix4f Roll {{1.0f, 0.0f, 0.0f, 0.0f},
                          {0.0f, (float) cos(roll), - (float) sin(roll), 0.0f},
                          {0.0f, (float) sin(roll), (float) cos(roll), 0.0f},
                          {0.0f, 0.0f, 0.0f, 1.0f}};
    

    Eigen::Matrix4f Pitch {{(float) cos(pitch), 0.0f, (float) sin(pitch), 0.0f},
                           {0.0f, 1.0f, 0.0f, 0.0f},
                           {- (float) sin(pitch), 0.0f, (float) cos(pitch), 0.0f},
                           {0.0f, 0.0f, 0.0f, 1.0f}};

    
    Eigen::Matrix4f Yaw {{(float) cos(yaw), - (float) sin(yaw), 0.0f, 0.0f},
                         {(float) sin(yaw), (float) cos(yaw), 0.0f, 0.0f},
                         {0.0f, 0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 1.0f}};

    return (Roll*Pitch)*Yaw;
}

Eigen::Matrix4f RotateTranslate(Eigen::Matrix4f rotation, Eigen::Vector3f position){
    Eigen::Matrix4f trans;
    trans << Eigen::Matrix4f::Identity();
    trans.block(0,3,3,1) << position;
    return rotation*trans;
}

Eigen::Vector3f TransformVector(Eigen::Vector3f xyz_coord, Eigen::Matrix4f rotation, Eigen::Vector3f translation){
    Eigen::Vector4f xyz_vec(0.0f, 0.0f, 0.0f, 1.0f);
    xyz_vec.head<3>() = xyz_coord;
    Eigen::Vector4f Transformed = RotateTranslate(rotation, translation)*xyz_vec;
    return Transformed.head<3>();
}
