# Writeup
Here follows a description of how each of the rubrics required for the **Control of a 3D Quadrotor project** were achived.<p>
This project is part of the Udacity Flying Car and Autonomous Flight Engineer Nanodegree Program
## Implement body rate control in C++.
The body rate controller receives the actual and target body rates and outputs the required torque that should be applied on each axis.<p>
A P controller is used to calculate the desired rotational acceleration for each axis.
```
V3F pqrError = pqrCmd - pqr;
V3F uBar = kpPQR * pqrError;
```
That rotational acceleration is then multiplied by the momentum of each axis to calculate the torque to be applied.
```
V3F I = V3F(Ixx, Iyy, Izz);
momentCmd = I * uBar;
```

## Implement roll pitch control in C++.
Given the collective thrust to be applied and knowing the mass of the quadrotor, we can calculate the acceleration experienced by it
```
float acc = -collThrustCmd / mass;
```
We may then determine the required roll and pitch angles, without forgetting to restrict the angles due to physical limitations 
```
float b_x_c = accelCmd.x / acc;
b_x_c = CONSTRAIN(b_x_c, -maxTiltAngle, maxTiltAngle);

float b_y_c = accelCmd.y / acc;
b_y_c = CONSTRAIN(b_y_c, -maxTiltAngle, maxTiltAngle);
```
A P controller is used to calculate the roll and pitch rates required to maintain the desired roll and pitch angles
```
float b_x_a = R(0,2);
float b_x_err = b_x_c - b_x_a;
float b_x_c_dot = kpBank * b_x_err;

float b_y_a = R(1,2);
float b_y_err = b_y_c - b_y_a;
float b_y_c_dot = kpBank * b_y_err;
```
## Implement altitude controller in C++.
First the required velocity has to be restricted due to hardware limitations.
```
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
```
A PID controller is used to calculate the required acceleration to keep the desired altitude
```
const float posZErr = posZCmd - posZ;
const float velZErr = velZCmd - velZ;

const float p = kpPosZ * posZErr;
integratedAltitudeError += posZErr * dt;
const float i = KiPosZ * integratedAltitudeError;
const float d = kpVelZ * velZErr;
const float u1Bar = p + i + d + accelZCmd;
```
Finally the required thrust is calculated taking into account the required rotational acceleration as well the drone's mass and yaw.
```
const auto c = static_cast<const float>((u1Bar - CONST_GRAVITY) / R(2, 2));
thrust = - mass * (u1Bar - (float)CONST_GRAVITY) / R(2,2);
```  
## Implement lateral position control in C++.
First the velocity is restricted due to hardware limitations
```
velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY); 
```
A PD controller is used to calculate the required acceleration to keep the desired position.
```
V3F posErr = posCmd - pos;
V3F velErr = velCmd - vel;
accelCmd = kpPosXY * posErr + kpVelXY * velErr + accelCmd;
```
Finally this acceleration also has to be restricted due to hardware limitations
```
accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
```
## Implement yaw control in C++.
A P controller is used to calculate the yaw rate required to keep the desired yaw. The yaw angle error is restricted between 0 and 2 * Pi befeore being used as input to the controller.
```
float yawErr = yawCmd - yaw;
yawErr = fmodf(yawErr, 2 * F_PI);
yawRateCmd = kpYaw * yawErr;
```
## Implement calculating the motor commands given commanded thrust and moments in C++.
By drawing a force and torque diagram, we can come up with the following system of equations
```
tauX = (F1 - F2 - F3 + F4) * d
tauY = (F1 + F2 - F3 + F4) * d
tauZ = (F1 - F2 + F3 - F4) * kappa
```
This system can be solved and written as
```
float Fx = momentCmd.x / d;
float Fy = momentCmd.y / d;
float Fz = momentCmd.z / kappa;

cmd.desiredThrustsN[0] = (collThrustCmd + Fx + Fy - Fz) / 4.f; // front left
cmd.desiredThrustsN[1] = (collThrustCmd - Fx + Fy + Fz) / 4.f; // front right
cmd.desiredThrustsN[2] = (collThrustCmd + Fx - Fy + Fz) / 4.f; // rear left
cmd.desiredThrustsN[3] = (collThrustCmd - Fx - Fy - Fz) / 4.f; // rear right
```