#ifndef __trajectories__
#define __trajectories__

#include <cmath>
#include <math.h>
#include <iostream>

#define POLYNOMIAL      0
#define CIRCLE          1
#define CUAD            2

float p5_rt(float x);
float circle_rt(float x);
float cuad_rt(float x);

// Function pointer type
typedef float (*FunctionPtr)(float);

class Trajectories {
public:
    Trajectories();
    
    Trajectories(uint8_t mytype);

    float get_value(float x);

private:
    uint8_t ttype_;
    FunctionPtr funct_rt;
};


#endif