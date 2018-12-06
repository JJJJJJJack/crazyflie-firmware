
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "ando.h"

#include "log.h"
#include "param.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define MIN_THRUST  1000
#define MAX_THRUST  60000

static bool tiltCompensationEnabled = false;
// Test GE parameters
static bool GE_THRUST_TEST = false;
static uint16_t ge_thrust;

// ACRO parameters
static bool PURE_YAWRATE = false, DISABLE_YAW = false;
static bool RollFlip = false, PitchFlip = false;
static uint16_t FlipCount = 0, FlipNumber = 5;
static float ACROT = 1.5f;
static float maxRate = 1000.0f;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

void stateControllerInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  ANDOInit(ATTITUDE_UPDATE_DT);
}

bool stateControllerTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

float flipDesiredOmega(float time)
{
  float desiredRate = 0.0f;
  float Tacc = ACROT/2.0f;
  float Tflip = ACROT - Tacc;
  if (time <= Tacc)
    desiredRate = 0.0f;
  else if (time <= Tacc + Tflip)
    desiredRate = maxRate;
  /*float Ta = ACROT - 360.0f*(float)FlipNumber/maxRate;
  float Tb = 720.0f*(float)FlipNumber/maxRate - ACROT;
  if (time <= Ta)
    desiredRate = maxRate/Ta*time;
  else if (time <= Ta+Tb)
    desiredRate = maxRate;
  else
  desiredRate = maxRate - maxRate/Ta*(time-Ta-Tb);*/
  return desiredRate;
}

float flipDesiredThrust(float time)
{
  float desiredThrust = 0.0f;
  float Tacc = ACROT/2.0f;
  if (time <= Tacc)
    desiredThrust = 40000.0f;
  else
    desiredThrust = 33000.0f;
  return desiredThrust;
}

void stateController(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       attitudeDesired.yaw -= setpoint->attitudeRate.yaw/500.0f;
      while (attitudeDesired.yaw > 180.0f)
        attitudeDesired.yaw -= 360.0f;
      while (attitudeDesired.yaw < -180.0f)
        attitudeDesired.yaw += 360.0f;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }
    if (setpoint->mode.yaw == modeVelocity && PURE_YAWRATE) {
      rateDesired.yaw = setpoint->attitudeRate.yaw;
      attitudeControllerResetYawAttitudePID();
    }
    if (DISABLE_YAW) {
      rateDesired.yaw = 0.f;
      attitudeControllerResetYawAttitudePID();
    }

    // Only do acro when it is in angle mode so we can get back to angle mode
    if (setpoint->mode.roll == modeAbs && setpoint->mode.pitch == modeAbs) {
      if (RollFlip || PitchFlip) {
	DISABLE_YAW = true;
	float time = (float)FlipCount++ * ATTITUDE_UPDATE_DT;
	float Tacc = ACROT/2.0f;
	if (time >= Tacc){
	  if (RollFlip){
	    rateDesired.roll = flipDesiredOmega(time);//Desired_omega
	    rateDesired.pitch = 0.0f;
	  }else{
	    rateDesired.pitch = flipDesiredOmega(time);//Desired_omega
	    rateDesired.roll = 0.0f;
	  }
	}
	FlipCount += 1;
	actuatorThrust = flipDesiredThrust(time);//Desired_thrust
	if (time >= ACROT) {
	  RollFlip = false;
	  PitchFlip = false;
	  attitudeControllerResetAllPID();
	  DISABLE_YAW = false;
	  actuatorThrust = setpoint->thrust;
	  // Reset flip count for next flip
	  FlipCount = 0;
	}
      }
    }
    
    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    
    if (DISABLE_YAW) {
      control->yaw = 0;
    }
    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  control_t compensate_control;
  ANDO(sensors, state, control, &compensate_control);

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();
    ANDOReset();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }

  control->thrust -= compensate_control.thrust;
  control->roll   -= compensate_control.roll;
  control->pitch  -= compensate_control.pitch;
  control->yaw    -= compensate_control.yaw;
  
  if (GE_THRUST_TEST != false){
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    if (ge_thrust > MIN_THRUST && ge_thrust < MAX_THRUST)
      control->thrust = ge_thrust;
    else if (ge_thrust == 0)
      // Pass through the setpoint thrust in GE test and no thrust manually set
      control->thrust = setpoint->thrust;
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(GE_ANDO)
PARAM_ADD(PARAM_UINT8,  tiltComp,      &tiltCompensationEnabled)
PARAM_ADD(PARAM_UINT8,  GETEST_ENABLE, &GE_THRUST_TEST)
PARAM_ADD(PARAM_UINT16, GE_thrust,     &ge_thrust)
PARAM_ADD(PARAM_UINT8,  pureYawRate,   &PURE_YAWRATE)
PARAM_ADD(PARAM_UINT8,  yaw_disable,   &DISABLE_YAW)
PARAM_GROUP_STOP(GE_ANDO)

PARAM_GROUP_START(ACRO)
PARAM_ADD(PARAM_UINT8,   roll_flip,   &RollFlip)
PARAM_ADD(PARAM_UINT8,   pitch_flip,  &PitchFlip)
PARAM_ADD(PARAM_FLOAT,   maxRate,     &maxRate)
PARAM_ADD(PARAM_FLOAT,   flipTime,    &ACROT)
PARAM_ADD(PARAM_UINT16,  flipNumber,  &FlipNumber)
PARAM_GROUP_STOP(ACRO)
