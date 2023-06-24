// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "lib/StickLinear.h"
#include <math.h>

double StickLinear::Linearize(double value, double min, double max, double offset)
{
    double rtn = value - offset;
    rtn = (rtn > 0) ? rtn * 1/fabs(max-offset) : rtn * 1/fabs(min-offset);
    return rtn;
}

StickLinear::StickLinear() = default;
