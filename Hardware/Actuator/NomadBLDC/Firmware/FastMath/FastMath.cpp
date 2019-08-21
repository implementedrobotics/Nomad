#include "FastMath.h"
#include "LUT.h"

const float Multiplier = 81.4873308631f;

float FastMath::FastSin(float theta){
    while (theta < 0.0f) theta += 6.28318530718f;
    while (theta >= 6.28318530718f) theta -= 6.28318530718f;    
    return SinTable[(int) (Multiplier*theta)] ;
    }
    
float FastMath::FastCos(float theta){
    return FastSin(1.57079632679f - theta);
    }