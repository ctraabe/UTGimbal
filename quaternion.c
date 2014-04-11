#include "quaternion.h"

#include <math.h>

float RollAngleFromQuaternion(float q[4], float q0_squared)
{
  return atan2(2 * (q[0] * q[1] + q[2] * q[3]),
    2 * (q0_squared + q[3] * q[3]) - 1);
}

float PitchAngleFromQuaternion(float q[4])
{
  return asin(2 * (q[0] * q[2] - q[1] * q[3]));
}

float YawAngleFromQuaternion(float q[4], float q0_squared)
{
  return atan2(2 * (q[0] * q[3] + q[1] * q[2]),
    2 * (q0_squared + q[1] * q[1]) - 1);
}