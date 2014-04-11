#ifndef QUATERNION_H
#define QUATERNION_H

// Note that floating point math takes many CPU cycles and should be voided
// wherever possible. Therefore, for efficiency, q[0] * q[0] can be reused when
// computing roll or yaw angles to save computation.
float RollAngleFromQuaternion(float q[4], float q0_squared);
float PitchAngleFromQuaternion(float q[4]);
float YawAngleFromQuaternion(float q[4], float q0_squared);

#endif //QUATERNION_H
