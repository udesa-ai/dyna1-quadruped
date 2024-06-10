#ifndef __bezier__
#define __bezier__

/*
Bezier Curves from: https://dspace.mit.edu/handle/1721.1/98270
Rotation Logic from: http://www.inase.org/library/2014/santorini/bypaper/ROBCIRC/ROBCIRC-54.pdf
*/

#include "algebra.hpp"
#include <array>
#include "quad_kinematics.hpp"
#include <random>
#include <cmath>

#define STANCE      0
#define SWING       1

class BezierGait {
public:
    BezierGait();

    BezierGait(std::array<float, 4> dSref, std::array<float, 4> dSref_end, float dt, float Tswing_ref);

    void reset();
    
    void GetPhase(uint8_t index, float Tstance, float Tswing, float* Phase, uint8_t* StanceSwing);
    
    float Get_ti(uint8_t index, float Tstride);
    
    void Increment(float dt, float Tstride, float Tswing);
    
    void CheckTouchDown();
    
    float BernSteinPoly(float t, uint8_t k, float point);
    
    float Binomial(uint8_t k);

    uint16_t Binomial_11(uint8_t k);
    
    Eigen::Vector3f BezierSwing(float phase, float L, float LateralFraction, float clearance_height);
    
    Eigen::Vector3f SineStance(float phase, float L, float LateralFraction, float penetration_depth);
    
    float YawCircle(Eigen::Vector3f T_bf, uint8_t index);
    
    Eigen::Vector3f SwingStep(float phase, float L, float LateralFraction,
                              float YawRate, float clearance_height,
                              Eigen::Vector3f T_bf, uint8_t index);
    
    Eigen::Vector3f StanceStep(float phase, float L, float LateralFraction,
                              float YawRate, float penetration_depth,
                              Eigen::Vector3f T_bf, uint8_t index);
    
    Eigen::Vector3f GetFootStep(float L, float LateralFraction, float YawRate,
                                float clearance_height, float penetration_depth,
                                float Tstance, Eigen::Vector3f T_bf, uint8_t index, float Tswing);
    
    TransfDict GenerateTrajectory(float L, float LateralFraction, float YawRate,
                                  float vel, TransfDict T_bf_, float clearance_height,
                                  float penetration_depth, float dt);
                                  
    std::array<float, 4> dSref;
    std::array<float, 4> dSref_end;
    float dt;
    float Tswing_ref;
    bool use_dsref;


private:
    
    MatrixJoint Prev_fxyz;
    uint8_t NumControlPoints;
    
    float time;
    float TD_time;
    float time_since_last_TD;
    uint8_t StanceSwing_;
    float SwRef;
    float Stref;
    
    uint8_t ref_idx;
    std::array<float, 4> Phases;
    uint16_t binomial_11_values[12];
};


#endif
