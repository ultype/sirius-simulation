#include "math_utility.hh"

///////////////////////////////////////////////////////////////////////////////
// Returns the sign of the variable
// Example: value_signed=value*sign(variable)
// 010824 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int sign(const double &variable)
{
    int sign = 0;
    if (variable < 0.)
        sign = -1;
    if (variable >= 0.)
        sign = 1;

    return sign;
}

