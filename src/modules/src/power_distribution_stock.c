/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "motors.h"

static bool motorSetEnable = false;
static bool EnableServo = false, EnablePartialGE = false;
int16_t ServoLeftK = 1000;
int16_t ServoRightK = 1000;
float ServoLeftAngle = 0, ServoRightAngle = 0;
float PartialGEm1 = 1.0, PartialGEm2 = 1.0, PartialGEm3 = 1.0, PartialGEm4 = 1.0;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
  motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control, setpoint_t *setpoint)
{
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    if(EnableServo){
      /*
      	use output port M1 M4 for motor control
      	 4  1
      	(3  2)
      	Port M2 M3 are used for servo control
       */
      motorPower.m1 = limitThrust(control->thrust);
      motorsSetRatio(MOTOR_M1, motorPower.m1);
      motorPower.m4 = limitThrust(control->thrust);
      motorsSetRatio(MOTOR_M4, motorPower.m4);


      motorPower.m2 = limitThrust(32767 + ServoRightK * (int16_t)ServoRightAngle);
      motorsSetRatio(MOTOR_M2, motorPower.m2);
      motorPower.m3 = limitThrust(32767 + ServoLeftK * (int16_t)ServoLeftAngle);
      motorsSetRatio(MOTOR_M3, motorPower.m3);
    }else if(EnablePartialGE){
      PartialGEm1 = PartialGEm1<1?1:PartialGEm1;
      PartialGEm2 = PartialGEm2<1?1:PartialGEm2;
      PartialGEm3 = PartialGEm3<1?1:PartialGEm3;
      PartialGEm4 = PartialGEm4<1?1:PartialGEm4;
      motorPower.m1 = limitThrust((int32_t)((float)motorPower.m1/PartialGEm1));
      motorPower.m2 = limitThrust((int32_t)((float)motorPower.m2/PartialGEm2));
      motorPower.m3 = limitThrust((int32_t)((float)motorPower.m3/PartialGEm3));
      motorPower.m4 = limitThrust((int32_t)((float)motorPower.m4/PartialGEm4));
      motorsSetRatio(MOTOR_M1, motorPower.m1);
      motorsSetRatio(MOTOR_M2, motorPower.m2);
      motorsSetRatio(MOTOR_M3, motorPower.m3);
      motorsSetRatio(MOTOR_M4, motorPower.m4);
    }else{
      motorsSetRatio(MOTOR_M1, motorPower.m1);
      motorsSetRatio(MOTOR_M2, motorPower.m2);
      motorsSetRatio(MOTOR_M3, motorPower.m3);
      motorsSetRatio(MOTOR_M4, motorPower.m4);
    }
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(servoSet)
PARAM_ADD(PARAM_UINT8, servoEnable, &EnableServo)
PARAM_ADD(PARAM_FLOAT, servoLeftAngle,   &ServoLeftAngle)
PARAM_ADD(PARAM_FLOAT, servoRightAngle,  &ServoRightAngle)
PARAM_GROUP_STOP(servoSet)

PARAM_GROUP_START(partialGE)
PARAM_ADD(PARAM_UINT8, partialEnable, &EnablePartialGE)
PARAM_ADD(PARAM_FLOAT, partialGEm1,   &PartialGEm1)
PARAM_ADD(PARAM_FLOAT, partialGEm2,   &PartialGEm2)
PARAM_ADD(PARAM_FLOAT, partialGEm3,   &PartialGEm3)
PARAM_ADD(PARAM_FLOAT, partialGEm4,   &PartialGEm4)
PARAM_GROUP_STOP(partialGE)

LOG_GROUP_START(partialGE)
LOG_ADD(LOG_FLOAT, m1, &PartialGEm1)
LOG_ADD(LOG_FLOAT, m2, &PartialGEm2)
LOG_ADD(LOG_FLOAT, m3, &PartialGEm3)
LOG_ADD(LOG_FLOAT, m4, &PartialGEm4)
LOG_GROUP_STOP(partialGE)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)
