#include "trajectories.hpp"

float p5_rt(float x) {
    float y = 10*(pow(x, 3)) - 15*(pow(x, 4)) + 6*(pow(x, 5));
    return y;
}

float circle_rt(float x) {
    float y = x - (1/(2 * M_PI)) * sin(2 * M_PI * x);
    return y;
}

float cuad_rt(float x) {
    float y = pow(x, 1.75f);
    return y;
}

Trajectories::Trajectories() {}

Trajectories::Trajectories(uint8_t mytype) {

    ttype_ = mytype;
    if (ttype_ == POLYNOMIAL) {
        funct_rt = p5_rt;
    } else if (ttype_ == CIRCLE) {
        funct_rt = circle_rt;
    } else if (ttype_ == CUAD) {
        funct_rt = cuad_rt;
    }

    }

float Trajectories::get_value(float x) {
    return funct_rt(x);
}


