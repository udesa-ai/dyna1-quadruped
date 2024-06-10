#include "bezier.hpp"

BezierGait::BezierGait() {}

BezierGait::BezierGait(std::array<float, 4> dSref, std::array<float, 4> dSref_end, float dt, float Tswing_ref)
    : dSref(dSref), dSref_end(dSref_end), dt(dt), Tswing_ref(Tswing_ref)
    {
        binomial_11_values[0] = 1;
        binomial_11_values[1] = 11;
        binomial_11_values[2] = 55;
        binomial_11_values[3] = 165;
        binomial_11_values[4] = 330;
        binomial_11_values[5] = 462;
        binomial_11_values[6] = 462;
        binomial_11_values[7] = 330;
        binomial_11_values[8] = 165;
        binomial_11_values[9] = 55;
        binomial_11_values[10] = 11;
        binomial_11_values[11] = 1;

        Prev_fxyz << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
        
        NumControlPoints = 11;

        time = 0.0f;

        TD_time = 0.0f;
        
        time_since_last_TD = 0.0f;

        StanceSwing_ = SWING;

        SwRef = 0.0f;
        Stref = 0.0f;

        ref_idx = 0;

        Phases = dSref;

        use_dsref = false;

        if (dSref_end[0] != 0.0f) {
            use_dsref = true;
        }
    }

void BezierGait::reset() 
{
    for (uint8_t i = 0; i < 4; i++)
    {
        Prev_fxyz(i) = 0.0f;
    }

    time = 0.0f;

    TD_time = 0.0f;

    time_since_last_TD = 0.0f;

    StanceSwing_ = SWING;

    SwRef = 0.0f;

    Stref = 0.0f;
}

void BezierGait::GetPhase(uint8_t index, float Tstance, float Tswing, float* Phase, uint8_t* StanceSwing)
{
    *StanceSwing = STANCE;
    float Sw_phase = 0.0f;
    float Tstride = Tstance + Tswing;
    float ti = Get_ti(index, Tstride);

    if (ti <= Tstance)
    {
        *StanceSwing = STANCE;
        float Stnphase;
        if (Tstance == 0.0f)
        {
            Stnphase = 0.0f;
        } else {
            Stnphase = ti / Tstance;
        } 

        if (index == ref_idx){
            StanceSwing_ = *StanceSwing;
        }
        *Phase = Stnphase;   
        return ;
    } else {
        *StanceSwing = SWING;
        Sw_phase = (ti - Tstance) / Tswing;
    }

    if (index == ref_idx) {
        StanceSwing_ = *StanceSwing;
        SwRef = Sw_phase;
    }

    *Phase = Sw_phase;
}

float BezierGait::Get_ti(uint8_t index, float Tstride)
{
    float division = (time_since_last_TD + dSref[index] * Tstride)/Tstride;
    return (division - std::floor(division))*Tstride;
}

void BezierGait::Increment(float dt, float Tstride, float Tswing)
{
    CheckTouchDown();    
    time += dt;
    time_since_last_TD = time - TD_time;

    if (Tstride < Tswing + dt) {
        time = 0.0f;
        time_since_last_TD = 0.0f;
        TD_time = 0.0f;
        SwRef = 0.0f;
    }
}
    
void BezierGait::CheckTouchDown()
{
    if (SwRef >= 0.99) {
        TD_time = time;
        SwRef = 0.0f;
    }
        
}

float BezierGait::BernSteinPoly(float t, uint8_t k, float point)
{
    return point * Binomial_11(k) * pow(t, k) * pow(1 - t, NumControlPoints - k);
}

float BezierGait::Binomial(uint8_t k)
{
    float w = 1;
    for (int i = NumControlPoints - k + 1; i <= NumControlPoints; ++i) w = w * i;
    for (int i = 1; i <= k; ++i) w = w / i;
    return w;
}

uint16_t BezierGait::Binomial_11(uint8_t k)
{
    return binomial_11_values[k];
}

Eigen::Vector3f BezierGait::BezierSwing(float phase, float L, float LateralFraction, float clearance_height)
{
    float X_polar = cos(LateralFraction);
    float Y_polar = sin(LateralFraction);

    Eigen::Vector<float, 12> Step {-L,
                                   -L * 1.4f,
                                   -L * 1.5f,
                                   -L * 1.5f,
                                   -L * 1.5f,
                                   0.0f,
                                   0.0f,
                                   0.0f,
                                   L * 1.5f,
                                   L * 1.5f,
                                   L * 1.4f,
                                   L};

    
    Eigen::Vector<float, 12> x = Step * X_polar;
    Eigen::Vector<float, 12> y = Step * Y_polar;

    
    Eigen::Vector<float, 12> z = {0.0f,
                                   0.0f,
                                   clearance_height * 0.9f,
                                   clearance_height * 0.9f,
                                   clearance_height * 0.9f,
                                   clearance_height * 0.9f,
                                   clearance_height * 0.9f,
                                   clearance_height * 1.1f,
                                   clearance_height * 1.1f,
                                   clearance_height * 1.1f,
                                   0.0f,
                                   0.0f};
    
    float stepX = 0.0f;
    float stepY = 0.0f;
    float stepZ = 0.0f;
    
    for (uint8_t i = 0; i < 12; i++) {
        stepX += BernSteinPoly(phase, i, x(i));
        stepY += BernSteinPoly(phase, i, y(i));
        stepZ += BernSteinPoly(phase, i, z(i));
    }

    Eigen::Vector3f answer { (float) stepX, (float) stepY, (float) stepZ};

    return answer;
}

Eigen::Vector3f BezierGait::SineStance(float phase, float L, float LateralFraction, \
                    float penetration_depth)
{
    float x_polar = cos(LateralFraction);
    float y_polar = sin(LateralFraction);

    float step = L * (1.0 - 2.0 * phase);
    float stepX = step * x_polar;
    float stepY = step * y_polar;
    float stepZ = 0.0f;
    if (L != 0.0f) {
        stepZ = -penetration_depth * cos((M_PI * (stepX + stepY)) / (2.0 * L));
    }

    Eigen::Vector3f answer { (float) stepX, (float) stepY, (float) stepZ};
    return answer;
}

float BezierGait::YawCircle(Eigen::Vector3f T_bf, uint8_t index)
{
    float DefaultBodyToFoot_Magnitude = sqrt(pow(T_bf(0),2) + pow(T_bf(1),2));
    
    float DefaultBodyToFoot_Direction = atan2(T_bf(1), T_bf(0));
    Eigen::Vector3f g_xyz = Prev_fxyz.block(index,0,1,3).transpose() - T_bf;

    float g_mag = sqrt(pow(g_xyz(0),2) + pow(g_xyz(1),2));
    float th_mod = atan2(g_mag, DefaultBodyToFoot_Magnitude);

    float phi_arc = 0;
    if (index == 1 || index == 2){
        phi_arc = M_PI / 2.0 + DefaultBodyToFoot_Direction + th_mod;
    } else {
        phi_arc = M_PI / 2.0 - DefaultBodyToFoot_Direction + th_mod;
    }

    return phi_arc;
}

Eigen::Vector3f BezierGait::SwingStep(float phase, float L, float LateralFraction,
                            float YawRate, float clearance_height,
                            Eigen::Vector3f T_bf, uint8_t index)
{
    float phi_arc = YawCircle(T_bf, index);
    
    Eigen::Vector3f delta_lin = BezierSwing(phase, L, LateralFraction, clearance_height);

    float X_delta_lin = delta_lin(0);
    float Y_delta_lin = delta_lin(1);
    float Z_delta_lin = delta_lin(2);

    Eigen::Vector3f delta_rot = BezierSwing(phase, YawRate, phi_arc, clearance_height);

    float X_delta_rot = delta_rot(0);
    float Y_delta_rot = delta_rot(1);
    float Z_delta_rot = delta_rot(2);

    Eigen::Vector3f coord = {X_delta_lin + X_delta_rot,
                             Y_delta_lin + Y_delta_rot,
                             Z_delta_lin + Z_delta_rot};

    Prev_fxyz.block(index,0,1,3) << coord.transpose();

    return coord;
}

Eigen::Vector3f BezierGait::StanceStep(float phase, float L, float LateralFraction,
                            float YawRate, float penetration_depth,
                            Eigen::Vector3f T_bf, uint8_t index)
{
    float phi_arc = YawCircle(T_bf, index);

    Eigen::Vector3f delta_lin = SineStance(
        phase, L, LateralFraction, penetration_depth);

    float X_delta_lin = delta_lin(0);
    float Y_delta_lin = delta_lin(1);
    float Z_delta_lin = delta_lin(2);

    Eigen::Vector3f delta_rot = SineStance(phase, YawRate, phi_arc, penetration_depth);

    float X_delta_rot = delta_rot(0);
    float Y_delta_rot = delta_rot(1);
    float Z_delta_rot = delta_rot(2);

    Eigen::Vector3f coord = {X_delta_lin + X_delta_rot,
                             Y_delta_lin + Y_delta_rot,
                             Z_delta_lin + Z_delta_rot};
    
    Prev_fxyz.block(index,0,1,3) << coord.transpose();

    return coord;
}

Eigen::Vector3f BezierGait::GetFootStep(float L, float LateralFraction, float YawRate,
                            float clearance_height, float penetration_depth,
                            float Tstance, Eigen::Vector3f T_bf, uint8_t index, float Tswing)
{
    float phase = 0;
    uint8_t StanceSwing = 0;

    GetPhase(index, Tstance, Tswing, &phase, &StanceSwing);

    float stored_phase = 0;
    if (StanceSwing == SWING){
        stored_phase = phase + 1.0;
    } else {
        stored_phase = phase;
    }

    Phases[index] =  stored_phase;
    if (StanceSwing == STANCE) {
        return StanceStep(phase, L, LateralFraction, YawRate,
                                penetration_depth, T_bf, index);
    } else if (StanceSwing == SWING) {
        return SwingStep(phase, L, LateralFraction, YawRate,
                                clearance_height, T_bf, index);
    }
    
    Eigen::Vector3f empty = {0,0,0};

    return empty;
}

TransfDict BezierGait::GenerateTrajectory(float L, float LateralFraction, float YawRate,
                                float vel, TransfDict T_bf_, float clearance_height,
                                float penetration_depth, float dt)
{

    float Tstance = 0;
    if (vel != 0.0f){
        Tstance = 2.0f * fabs(L) / fabs(vel);
    } else {
        Tstance = 0.0f;
        L = 0.0f;
        time = 0.0f;
        time_since_last_TD = 0.0f;
    }

    float Tswing = Tswing_ref;

    if (use_dsref) {
        Tswing = Tstance * (1 - dSref_end[0]) / dSref_end[0];
    }

    // if (Tswing < Tswing_ref) {
    //     Tstance = 1.3 * Tswing_ref;
    //     Tswing = Tswing_ref;
    // }

    YawRate *= dt;

    if (Tstance < dt) {
        Tstance = 0.0f;
        L = 0.0f;
        time = 0.0f;
        time_since_last_TD = 0.0f;
        YawRate = 0.0f;
    } /* else if (Tstance > 1.3 * Tswing) {
        Tstance = 1.3 * Tswing;
    } */

    Increment(dt, Tstance + Tswing, Tswing);
    TransfDict T_bf = T_bf_;
    
    uint8_t i = 0;
    for (const auto& pair : T_bf_) {
        uint8_t key = pair.first;
        Eigen::Matrix4f Tbf_in = pair.second;
        Eigen::Matrix3f R;
        Eigen::Vector3f p_bf;
        TransToRp(Tbf_in, &R, &p_bf);
        Eigen::Vector3f step_coord;
        if (Tstance > 0.0f) {
            step_coord = GetFootStep(L, LateralFraction, YawRate, clearance_height,
                                     penetration_depth, Tstance, p_bf, i, Tswing);
        } else {
            step_coord << 0.0f, 0.0f, 0.0f;
        }

        T_bf[key](0, 3) = Tbf_in(0, 3) + step_coord(0);
        T_bf[key](1, 3) = Tbf_in(1, 3) + step_coord(1);
        T_bf[key](2, 3) = Tbf_in(2, 3) + step_coord(2);

        
        ++i;
    }

    return T_bf;
}