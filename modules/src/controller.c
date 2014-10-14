/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <stdbool.h>
#include "math.h"
 
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "pid.h"
#include "param.h"
#include "imu.h"
#include "log.h"
/*
#define TRUNCATE_SINT16(out, in) \
  {\
    if (in > INT16_MAX) out = (int16_t)INT16_MAX;\
    else if (in < INT16_MIN) out = (int16_t)INT16_MIN;\
    else out = (int16_t)in;\
  }
*/

//Fancier version
#define TRUNCATE_SINT16(out, in) (out = (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

//Better semantic
#define SATURATE_SINT16(in) ( (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;
int16_t rollOutput1;
int16_t pitchOutput1;
int16_t yawOutput1;

static float q0Error;
static float q1Error;
static float q2Error;
static float q3Error;
static float q0Desired;
static float q1Desired;
static float q2Desired;
static float q3Desired;
static float kproll=50000.0;
static float kdroll=60.0;
static float kppitch=-22000.0;
static float kdpitch=40.0;
static float kpyaw=40000.0;
static float kdyaw=120.0;
static float Pr;
static float Pp;
static float Py;
static float Dr;
static float Dp;
static float Dy;
static float d=0.1;
static float h;
static float h1=1.0;

static bool isInit;

void controllerInit()
{
  if(isInit)
    return;
  
  //TODO: get parameters from configuration manager instead
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, IMU_UPDATE_DT);
  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
  
  isInit = true;
}

bool controllerTest()
{
  return isInit;
}

void controllerCorrectAttitudePIDTrue(
       float q0Actual, float q1Actual, float q2Actual, float q3Actual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired,
       float rollRateDotDesired, float pitchRateDotDesired, float yawRateDotDesired)
{
  float wddot[3]={rollRateDotDesired, pitchRateDotDesired, yawRateDotDesired};                 //  wd'      The desired angular acceleration vector
  float wd[3]={rollRateDesired, pitchRateDesired, yawRateDesired};                             //  wd       The desired angular velocity vector
  float w[3]={rollRateActual, pitchRateActual, yawRateActual};                                 //  w        The actual angular velocity vector
  int i,j;
  
  //kd1=0.5;


  q0Desired=cos(eulerRollDesired/2)*cos(eulerPitchDesired/2)*cos(eulerYawDesired)+sin(eulerRollDesired/2)*sin(eulerPitchDesired/2)*sin(eulerYawDesired);
  q1Desired=sin(eulerRollDesired/2)*cos(eulerPitchDesired/2)*cos(eulerYawDesired)-cos(eulerRollDesired/2)*sin(eulerPitchDesired/2)*sin(eulerYawDesired);
  q2Desired=cos(eulerRollDesired/2)*sin(eulerPitchDesired/2)*cos(eulerYawDesired)+sin(eulerRollDesired/2)*cos(eulerPitchDesired/2)*sin(eulerYawDesired);
  q3Desired=cos(eulerRollDesired/2)*cos(eulerPitchDesired/2)*sin(eulerYawDesired)-sin(eulerRollDesired/2)*sin(eulerPitchDesired/2)*cos(eulerYawDesired);

  //quaternion error
  q0Error = q0Desired*q0Actual - q1Desired*q1Actual - q2Desired*q2Actual - q3Desired*q3Actual;
  q1Error = q0Desired*q1Actual + q1Desired*q0Actual + q2Desired*q3Actual - q3Desired*q2Actual;
  q2Error = q0Desired*q2Actual - q1Desired*q3Actual + q2Desired*q0Actual + q3Desired*q1Actual;
  q3Error = q0Desired*q3Actual + q1Desired*q2Actual - q2Desired*q1Actual + q3Desired*q0Actual;

  // momentum of inertia
  float J[3][3]={{0.0082,0,0},
                 {0,0.0082,0},
                 {0,0,0.0164}};

 
  //Here we calculate the feedforward torque
  
  float Temp[3][3]={{1-2*(q3Error*q3Error+q2Error*q2Error), -2*q0Error*q3Error+2*q1Error*q2Error, 2*q0Error*q2Error+2*q3Error*q1Error},
             {2*q0Error*q3Error+2*q2Error*q1Error, 1-2*(q3Error*q3Error+q1Error*q1Error), -2*q0Error*q1Error+2*q2Error*q3Error},
             {-2*q0Error*q2Error+2*q3Error*q1Error, 2*q0Error*q1Error+2*q2Error*q3Error, 1-2*(q2Error*q2Error+q1Error*q1Error)}};                 //     R(qbar)                                       Rodrigues formula applied to the quaternion error 
  
  float RqbarT[3][3]={{Temp[0][0], Temp[1][0], Temp[2][0]},
                    {Temp[0][1], Temp[1][1], Temp[2][1]},
                    {Temp[0][2], Temp[1][2], Temp[2][2]}};                         //     R(qbar)t                                      The transpose of R(qbar)
  

  for (i=0; i<3; i++)
       {for(j=0; j<3; j++)
          {Temp[i][j]=J[i][0]*RqbarT[0][j]+J[i][1]*RqbarT[1][j]+J[i][2]*RqbarT[2][j];}
        };                                                                            //     J*R(qbar)t                                    The inertia matrix times R(qbar)t
  
  float TempVec[3];
  for (i=0; i<3; i++)
       { TempVec[i]=Temp[i][0]*wddot[0]+Temp[i][1]*wddot[1]+Temp[i][2]*wddot[2];
        };                                                                           //     J*R(qbar)t*wd'                                J*R(qbar)t times the desired angular acceleration
  
    float TempVec2[3];
  for (i=0; i<3; i++)
       { TempVec2[i]=RqbarT[i][0]*wd[0]+RqbarT[i][1]*wd[1]+RqbarT[i][2]*wd[2];
        };                                                                           //     R(qbar)t*wd                                   R(qbar)t times the desired angular velocity
  


  float wbar[3]={w[0]-TempVec2[0],w[1]-TempVec2[1],w[2]-TempVec[2]};                 //    wbar=w-R(qbar)t*wd
 
  float Temp1[3][3];
  for (i=0; i<3; i++)
       {for(j=0; j<3; j++)
           {if (j==i)
                Temp1[i][j]=0 ;
           }
        };
  Temp1[0][1]=-TempVec2[2];
  Temp1[0][2]=TempVec2[1];
  Temp1[1][0]=TempVec2[2];
  Temp1[1][2]=-TempVec2[0];
  Temp1[2][0]=-TempVec2[1];
  Temp1[2][1]=TempVec2[0];                                                             //     S[R(qbar)t*wd]                                The Skew symmetric matrix of R(qbar)t*wd
 

 for (i=0; i<3; i++)
       {for(j=0; j<3; j++)
          {Temp[i][j]=J[i][0]*RqbarT[0][j]+J[i][1]*RqbarT[1][j]+J[i][2]*RqbarT[2][j];}
        };

for (i=0; i<3; i++)
       { TempVec2[i]=Temp[i][0]*wd[0]+Temp[i][1]*wd[1]+Temp[i][2]*wd[2];
        };                                                                             //     J*R(qbar)t*wd                                 Inertia matrix times R(qbar)t times the desired angular velocity

float TempVec3[3];
for (i=0; i<3; i++)
       { TempVec3[i]=Temp1[i][0]*TempVec2[0]+Temp1[i][1]*TempVec2[1]+Temp1[i][2]*TempVec2[2];
        };                                                                             //     S[R(qbar)t*wd]*J*R(qbar)t*wd                  The product of the 2 terms above
float tff[3];
   
for (i=0; i<3; i++)
       { tff[i]=TempVec[i]+TempVec3[i];
        };                                                                            //     J*R(qbar)t*wd'+S[R(qbar)t*wd]*J*R(qbar)t*wd   The feedforward torque, given by the sum of J*R(qbar)t*wd' and S[R(qbar)t*wd]*J*R(qbar)t*wd

  
  //Here we calculate the feedback torque
  

  //RqbarTwddot=MatTimesVec(RqbarT,wddot);                                                                                        //    R(qbar)t*wd'
  //float wbardot[3]={rollRateDotDesired-RqbarTwddot[0], pitchRateDotDesired-RqbarTwddot[1], yawRateDotDesired-RqbarTwddot[2]};   //    wbar'=w'-R(qbar)t*wd'
  
  if (h1*q0Error>=-d)
      h=h1;
  else h=-h1;



  //float tfb[3]={-kp*h*q1Error-kd*wbar[0]-kd1*wbardot[0],-kp*h*q2Error-kd*wbar[1]-kd1*wbardot[1],-kp*h*q3Error-kd*wbar[2]-kd1*wbardot[2]}   //  tfb=-kp*h*[qbar(1);qbar(2);qbar(3)]-kd*wbar-kd1*wbar';
  float tfb[3]={-kproll*h*q1Error-kdroll*wbar[0],-kppitch*h*q2Error-kdpitch*wbar[1],-kpyaw*h*q3Error-kdyaw*wbar[2]};

  Pr=-kproll*h*q1Error;
  Dr=-kdroll*wbar[0];
  Pp=-kppitch*h*q2Error;
  Dp=-kdpitch*wbar[1];
  Py=-kpyaw*h*q3Error;
  Dy=-kdyaw*wbar[2];


  float tau[3]={tff[0]+tfb[0], tff[1]+tfb[1], tff[2]+tfb[2]};
  

  TRUNCATE_SINT16(rollOutput1, tau[0]);
  TRUNCATE_SINT16(pitchOutput1, tau[1]);
  TRUNCATE_SINT16(yawOutput1, tau[2]);
}

void controllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  TRUNCATE_SINT16(rollOutput, pidUpdate(&pidRollRate, rollRateActual, TRUE));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  TRUNCATE_SINT16(pitchOutput, pidUpdate(&pidPitchRate, pitchRateActual, TRUE));

  pidSetDesired(&pidYawRate, yawRateDesired);
  TRUNCATE_SINT16(yawOutput, pidUpdate(&pidYawRate, yawRateActual, TRUE));
}

void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, TRUE);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, TRUE);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, FALSE);
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput1;
  *pitch = pitchOutput1;
  *yaw = yawOutput1;
}


/*
LOG_GROUP_START(Derivative_Proportional_Parts)
LOG_ADD(LOG_FLOAT, PRoll, &Pr)
LOG_ADD(LOG_FLOAT, PPitch, &Pp)
LOG_ADD(LOG_FLOAT, PYaw, &Py)
LOG_ADD(LOG_FLOAT, DRoll, &Dr)
LOG_ADD(LOG_FLOAT, DPitch, &Dp)
LOG_ADD(LOG_FLOAT, DYaw, &Dy)
LOG_GROUP_STOP(Derivative_Proportional_Parts)
*/
LOG_GROUP_START(ReadQuat)
LOG_ADD(LOG_FLOAT, q0Desired, &q0Desired)
LOG_ADD(LOG_FLOAT, q1Desired, &q1Desired)
LOG_ADD(LOG_FLOAT, q2Desired, &q2Desired)
LOG_ADD(LOG_FLOAT, q3Desired, &q3Desired)
LOG_GROUP_STOP(ReadQuat)

LOG_GROUP_START(ReadQuatError)
LOG_ADD(LOG_FLOAT, q0Error, &q0Error)
LOG_ADD(LOG_FLOAT, q1Error, &q1Error)
LOG_ADD(LOG_FLOAT, q2Error, &q2Error)
LOG_ADD(LOG_FLOAT, q3Error, &q3Error)
LOG_GROUP_STOP(ReadQuatError)

LOG_GROUP_START(OldTorque)
LOG_ADD(LOG_FLOAT, roll, &rollOutput)
LOG_ADD(LOG_FLOAT, pitch, &pitchOutput)
LOG_ADD(LOG_INT16, yaw, &yawOutput)
LOG_GROUP_STOP(OldTorque)

PARAM_GROUP_START(hysteresis)
PARAM_ADD(PARAM_FLOAT, d, &d)
PARAM_ADD(PARAM_FLOAT, h, &h)
PARAM_GROUP_STOP(hysteresis)


PARAM_GROUP_START(SetKp)
PARAM_ADD(PARAM_FLOAT, KpRoll, &kproll)
PARAM_ADD(PARAM_FLOAT, KpPitch, &kppitch)
PARAM_ADD(PARAM_FLOAT, KpYaw, &kpyaw)
PARAM_GROUP_STOP(SetKp)

PARAM_GROUP_START(SetKd)
PARAM_ADD(PARAM_FLOAT, KdRoll, &kdroll)
PARAM_ADD(PARAM_FLOAT, KdPitch, &kdpitch)
PARAM_ADD(PARAM_FLOAT, KdYaw, &kdyaw)
PARAM_GROUP_STOP(SetKd)
/*
PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(pid_attitude)

PARAM_GROUP_START(pid_rate)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(pid_rate)
*/