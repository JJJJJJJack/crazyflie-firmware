#include "ando.h"
#include "log.h"
#include "param.h"
#include <math.h>

#define JX 1e-3f
#define JY 1e-3f
#define JZ 2e-2f
#define ARMLENGTH 0.12f
#define sq2 1.414213562373095f
#define gravity  9.81f
#define PLUS     0
#define MINUS    1
#define MULTIPLY 2

vec4 Z;
vec4 Zdot, G, Thrust, T, temp2, df;
vec4 dhat;

static float battery_factor = 1.0f;
static float LP_A = 300.0f;
static control_t _compensate_control, last_control_in = {0,0,0,0.0f}, last_control_out;
static control_t last_comp_in = {0,0,0,0.0f}, last_comp_out;
static Axis3f last_gyro_in, last_gyro_out;
static float last_vz_in = 0.0f, last_vz_out = 0.0f;
static vec4 ge_gain, _ge_gain={0.0f, 0.5f, 0.5f, 0.5f, 2.0f};
static float DT;
static float MOTOR_KA = -3.013e-14f, MOTOR_KB = 2.895e-09f, BATTERY_KI = 0.02f;
static bool ANDO_ENABLE = false, ANDO_CALC = false, _last_ANDO_CALC = false, ANDO_TORQUE = true;
static float MASS = 0.832f;

void VecInit(vec4 *vec) {
  vec->timestamp = 0;
  vec->x = 0;
  vec->y = 0;
  vec->z = 0;
  vec->w = 0;
}

void VecMath(vec4 a, vec4 b, vec4 *result, int sign) {
  switch(sign) {
  case PLUS:
    result->x = a.x + b.x;
    result->y = a.y + b.y;
    result->z = a.z + b.z;
    result->w = a.w + b.w;
    break;
  case MINUS:
    result->x = a.x - b.x;
    result->y = a.y - b.y;
    result->z = a.z - b.z;
    result->w = a.w - b.w;
    break;
  case MULTIPLY:
    result->x = a.x * b.x;
    result->y = a.y * b.y;
    result->z = a.z * b.z;
    result->w = a.w * b.w;
    break;
  default :
    result->x = 0;
    result->y = 0;
    result->z = 0;
    result->w = 0;
    break;
  }
}

void ANDOInit(float update_period) {
  ANDOReset();
  ge_gain = _ge_gain;
  DT = update_period;
}

void ANDOReset() {
  VecInit(&Z);
  VecInit(&Zdot);
  _compensate_control.thrust = 0;
  _compensate_control.roll = 0;
  _compensate_control.pitch = 0;
  _compensate_control.yaw = 0;
  battery_factor = 1.0f;
}

uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
    {
      value = UINT16_MAX;
    }
  else if(value < 0)
    {
      value = 0;
    }

  return (uint16_t)value;
}

void lowpassControl(control_t *input, control_t *last_in, control_t *last_out, control_t *output){
  if (last_in->thrust == 0 && last_in->roll == 0) {
    last_in = input;
    last_out = input;
  }
  output->thrust = ((2.0f-LP_A*DT)*last_out->thrust + LP_A*DT*(input->thrust+last_in->thrust))/(LP_A*DT+2.0f);
  output->roll   = ((2.0f-LP_A*DT)*last_out->roll   + LP_A*DT*((float)input->roll  +(float)last_in->roll))  /(LP_A*DT+2.0f);
  output->pitch  = ((2.0f-LP_A*DT)*last_out->pitch  + LP_A*DT*((float)input->pitch +(float)last_in->pitch)) /(LP_A*DT+2.0f);
  output->yaw    = ((2.0f-LP_A*DT)*last_out->yaw    + LP_A*DT*((float)input->yaw   +(float)last_in->yaw))   /(LP_A*DT+2.0f);
}

void lowpassGyro(Axis3f input, Axis3f *last_in, Axis3f *last_out, Axis3f *output){
  output->x = ((2.0f-LP_A*DT)*last_out->x + LP_A*DT*(input.x+last_in->x))/(LP_A*DT+2.0f);
  output->y = ((2.0f-LP_A*DT)*last_out->y + LP_A*DT*(input.y+last_in->y))/(LP_A*DT+2.0f);
  output->z = ((2.0f-LP_A*DT)*last_out->z + LP_A*DT*(input.z+last_in->z))/(LP_A*DT+2.0f);
}

void lowpassVz(float input, float last_in, float last_out, float *output){
  *output = ((2.0f-LP_A*DT)*last_out + LP_A*DT*(input+last_in))/(LP_A*DT+2.0f);
}

void ANDO(const sensorData_t *sensors, const state_t *state, control_t *control, control_t *compensate_control) {
  // Input: attitude, angular velocity(gyro), velocityZ, control effort
  //        inertia matrix, motor coefficient, and Z in R4
  // Output: zdot in R4
  // First filter the input
  control_t filtered_control = *control;
  lowpassControl(control, &last_control_in, &last_control_out, &filtered_control);
  last_control_in = *control;
  last_control_out = filtered_control;
  Axis3f filtered_gyro = sensors->gyro;
  lowpassGyro(sensors->gyro, &last_gyro_in, &last_gyro_out, &filtered_gyro);
  last_gyro_in = sensors->gyro;
  last_gyro_out = filtered_gyro;
  float filtered_vz = state->velocity.z;
  lowpassVz(state->velocity.z, last_vz_in, last_vz_out, &filtered_vz);
  last_vz_in = state->velocity.z;
  last_vz_out = filtered_vz;

  // zdot = -LZ + L(G-P) = part1 + part2
  // PART1
  VecInit(&temp2);
  // Reset the ANDO the moment the param ANDO_CALC is set to false
  if (_last_ANDO_CALC == true && ANDO_CALC == false)
    ANDOReset();
  if (ANDO_CALC == true)
  {
    float sphi = sin(state->attitude.roll/180.0f*(float)M_PI);
    float cphi = cos(state->attitude.roll/180.0f*(float)M_PI);
    float stheta = sin(state->attitude.pitch/180.0f*(float)M_PI);
    float ctheta = cos(state->attitude.pitch/180.0f*(float)M_PI);
    float gyrox = filtered_gyro.x/180.0f*(float)M_PI;
    float gyroy = filtered_gyro.y/180.0f*(float)M_PI;
    float gyroz = filtered_gyro.z/180.0f*(float)M_PI;

    vec4 part1;
    part1.x = -(ge_gain.x*Z.x + ge_gain.x*sphi*stheta/ctheta*Z.y + ge_gain.x*cphi*stheta/ctheta*Z.z);
    part1.y = -(ge_gain.y*cphi*Z.y - ge_gain.y*sphi*Z.z);
    part1.z = -(ge_gain.z*sphi/ctheta*Z.y + ge_gain.z*cphi/ctheta*Z.z);
    part1.w = -(ge_gain.w*cphi*ctheta*Z.w);
    vec4 GG, G_1, TPHI, PWM, con, temp1;
    PWM.x = filtered_control.thrust>0?limitThrust(filtered_control.thrust - filtered_control.roll/2 + filtered_control.pitch/2 + filtered_control.yaw):0;
    PWM.y = filtered_control.thrust>0?limitThrust(filtered_control.thrust - filtered_control.roll/2 - filtered_control.pitch/2 - filtered_control.yaw):0;
    PWM.z = filtered_control.thrust>0?limitThrust(filtered_control.thrust + filtered_control.roll/2 - filtered_control.pitch/2 + filtered_control.yaw):0;
    PWM.w = filtered_control.thrust>0?limitThrust(filtered_control.thrust + filtered_control.roll/2 + filtered_control.pitch/2 - filtered_control.yaw):0;
    // temp1 = pwm^2
    VecMath(PWM, PWM, &temp1, MULTIPLY);
    // temp2 = pwm^3
    VecMath(temp1, PWM, &temp2, MULTIPLY);
    con.x = battery_factor*MOTOR_KA; con.y = con.x; con.z = con.x; con.w = con.x;
    // temp2 = KA*PWM^3
    VecMath(temp2, con, &temp2, MULTIPLY);
    con.x = battery_factor*MOTOR_KB; con.y = con.x; con.z = con.x; con.w = con.x;
    // temp1 = KB*PWM^2
    VecMath(temp1, con, &temp1, MULTIPLY);
    VecMath(temp1, temp2, &Thrust, PLUS);
    T.x = ARMLENGTH*(-Thrust.x-Thrust.y+Thrust.z+Thrust.w);
    T.y = ARMLENGTH*( Thrust.x-Thrust.y-Thrust.z+Thrust.w);
    T.z = ARMLENGTH*( Thrust.x-Thrust.y+Thrust.z-Thrust.w)*sq2;
    T.w =           ( Thrust.x+Thrust.y+Thrust.z+Thrust.w);

    TPHI.x = -(JY-JZ)*( -((-gyroy)*cphi + (-gyroz)*ctheta*sphi)*((-gyroy)*sphi - (-gyroz)*cphi*ctheta));
    TPHI.y = -(JZ-JX)*(-(gyrox - (-gyroz)*stheta)*((-gyroy)*sphi - (-gyroz)*cphi*ctheta));
    TPHI.z = -(JX-JY)*((gyrox - (-gyroz)*stheta)*((-gyroy)*cphi + (-gyroz)*ctheta*sphi));
    TPHI.w = -MASS*(((-gyroy)*cphi + (-gyroz)*ctheta*sphi)*(filtered_vz*stheta) + (gyrox - (-gyroz)*stheta)*(filtered_vz*ctheta*sphi))*filtered_vz;

    G_1.x = 0;//-JX*(psidot*ctheta);
    G_1.y = -JY*((-gyroy)*sphi);
    G_1.z = -JZ*((-gyroy)*cphi);
    G_1.w = -MASS*(2*gyrox*ctheta*sphi - (-gyroy)*cphi*stheta - gyrox*ctheta*sphi - 2*gyrox*cphi*cphi*ctheta*sphi)*filtered_vz;

    GG.x = 0; GG.y = 0; GG.z = 0; GG.w = MASS*gravity*ctheta*cphi;
    VecMath(G_1, TPHI, &temp1, PLUS);
    VecMath(GG, T, &temp2, MINUS);
    VecMath(temp1, temp2, &G, PLUS);
    vec4 p;
    p.x = ge_gain.x*JX*gyrox;
    p.y = ge_gain.y*JY*(-gyroy);
    p.z = 0;
    p.w = MASS*filtered_vz;
    VecMath(G, p, &temp1, MINUS);
  
    vec4 part2;
    part2.x = ge_gain.x*temp1.x + ge_gain.x*sphi*stheta/ctheta*temp1.y + ge_gain.x*cphi*stheta/ctheta*temp1.z;
    part2.y = ge_gain.y*cphi*temp1.y - ge_gain.y*sphi*temp1.z;
    part2.z = ge_gain.z*sphi/ctheta*temp1.y + ge_gain.z*cphi/ctheta*temp1.z;
    part2.w = ge_gain.w*cphi*ctheta*temp1.w;
    // zdot = -LZ + L(G-p)
    Zdot.x = part1.x + part2.x;
    Zdot.y = part1.y + part2.y;
    Zdot.z = part1.z + part2.z;
    Zdot.w = part1.w + part2.w;

    // dhat = z + p
    VecMath(Z, p, &dhat, PLUS);
    // Integrate the battery factor if the dhat.w is negative
    if (dhat.w < 0 && battery_factor > 0.7f) {
      battery_factor += BATTERY_KI*dhat.w*DT;
    }
    if (ANDO_TORQUE == false){
      dhat.x = 0.0f;
      dhat.y = 0.0f;
      dhat.z = 0.0f;
    }

    // integrate Z according to runtime
    con.x = DT; con.y = DT; con.z = DT; con.w = DT;
    VecMath(Zdot, con, &temp1, MULTIPLY);
    VecMath(Z, temp1, &Z, PLUS);

    // Convert to PWM
    vec4 untie;
    untie.x = 0.25f/ARMLENGTH;
    untie.y = 0.25f/ARMLENGTH;
    untie.z = sq2*0.125f/ARMLENGTH;
    untie.w = 0.25f;
    df.x = -untie.x*dhat.x + untie.y*dhat.y + untie.z*dhat.z + untie.w*dhat.w;
    df.y = -untie.x*dhat.x - untie.y*dhat.y - untie.z*dhat.z + untie.w*dhat.w;
    df.z =  untie.x*dhat.x - untie.y*dhat.y + untie.z*dhat.z + untie.w*dhat.w;
    df.w =  untie.x*dhat.x + untie.y*dhat.y - untie.z*dhat.z + untie.w*dhat.w;
    df.x = df.x>0?df.x:0.f;
    df.y = df.y>0?df.y:0.f;
    df.z = df.z>0?df.z:0.f;
    df.w = df.w>0?df.w:0.f;
    //temp2.x = sqrt((df.x)/MOTOR_KFW);
    //temp2.y = sqrt((df.y)/MOTOR_KFW);
    //temp2.z = sqrt((df.z)/MOTOR_KFW);
    //temp2.w = sqrt((df.w)/MOTOR_KFW);
    float avg_pwm[4] = {0,0,0,0}, min_thrust[4] = {0,0,0,0}, max_thrust[4] = {50000,50000,50000,50000};
    float dfarray[4] = {df.x,df.y,df.z,df.w};
    float estimate = 0;
    for (int i=0; i<4; i++){
      for (int j=0; j<16; j++){
	avg_pwm[i] = (min_thrust[i]+max_thrust[i])/2;
	estimate = battery_factor*(MOTOR_KA*avg_pwm[i]*avg_pwm[i]*avg_pwm[i]+MOTOR_KB*avg_pwm[i]*avg_pwm[i]);
	if (estimate < dfarray[i])
	  min_thrust[i] = avg_pwm[i];
	else
	  max_thrust[i] = avg_pwm[i];
      }
    }
    temp2.x = avg_pwm[0];
    temp2.y = avg_pwm[1];
    temp2.z = avg_pwm[2];
    temp2.w = avg_pwm[3];
    control_t raw_comp;
    raw_comp.roll   = filtered_control.thrust>0 ? (int16_t)(0.25f*(-temp2.x - temp2.y + temp2.z + temp2.w)) : 0;
    raw_comp.pitch  = filtered_control.thrust>0 ? (int16_t)(0.25f*( temp2.x - temp2.y - temp2.z + temp2.w)) : 0;
    raw_comp.yaw    = filtered_control.thrust>0 ? (int16_t)(0.25f*( temp2.x - temp2.y + temp2.z - temp2.w)) : 0;
    raw_comp.thrust = filtered_control.thrust>0 ? 0.25f*( temp2.x + temp2.y + temp2.z + temp2.w) : 0;
    lowpassControl(&raw_comp, &last_comp_in, &last_comp_out, &_compensate_control);
    last_comp_in = raw_comp;
    last_comp_out = _compensate_control;
  }
  if (ANDO_ENABLE == true)
  {
    compensate_control->roll   = _compensate_control.roll;
    compensate_control->pitch  = _compensate_control.pitch;
    compensate_control->yaw    = _compensate_control.yaw;
    compensate_control->thrust = _compensate_control.thrust;
  }
  else
  {
    compensate_control->roll   = 0;
    compensate_control->pitch  = 0;
    compensate_control->yaw    = 0;
    compensate_control->thrust = 0;
  }
  _last_ANDO_CALC = ANDO_CALC;
}

LOG_GROUP_START(ANDO)
  LOG_ADD(LOG_FLOAT, Z1, &Z.x)
  LOG_ADD(LOG_FLOAT, Z2, &Z.y)
  LOG_ADD(LOG_FLOAT, Z3, &Z.z)
  LOG_ADD(LOG_FLOAT, Z4, &Z.w)
  LOG_ADD(LOG_FLOAT, pwm_thrust, &_compensate_control.thrust)
  LOG_ADD(LOG_INT16, pwm_roll,   &_compensate_control.roll)
  LOG_ADD(LOG_INT16, pwm_pitch,  &_compensate_control.pitch)
  LOG_ADD(LOG_INT16, pwm_yaw,    &_compensate_control.yaw)
  LOG_ADD(LOG_FLOAT, dhatx, &dhat.x)
  LOG_ADD(LOG_FLOAT, dhaty, &dhat.y)
  LOG_ADD(LOG_FLOAT, dhatz, &dhat.z)
  LOG_ADD(LOG_FLOAT, dhatw, &dhat.w)
LOG_GROUP_STOP(ANDO)

LOG_GROUP_START(ando_redundant)
  LOG_ADD(LOG_FLOAT, Zdotx, &Zdot.x)
  LOG_ADD(LOG_FLOAT, Zdoty, &Zdot.y)
  LOG_ADD(LOG_FLOAT, Zdotz, &Zdot.z)
  LOG_ADD(LOG_FLOAT, Zdotw, &Zdot.w)
  LOG_ADD(LOG_FLOAT, Thrustx, &Thrust.x)
  LOG_ADD(LOG_FLOAT, Thrusty, &Thrust.y)
  LOG_ADD(LOG_FLOAT, Thrustz, &Thrust.z)
  LOG_ADD(LOG_FLOAT, Thrustw, &Thrust.w)
  LOG_ADD(LOG_FLOAT, dfx, &df.x)
  LOG_ADD(LOG_FLOAT, dfy, &df.y)
  LOG_ADD(LOG_FLOAT, dfz, &df.z)
  LOG_ADD(LOG_FLOAT, dfw, &df.w)
  LOG_ADD(LOG_FLOAT, t2x, &temp2.x)
  LOG_ADD(LOG_FLOAT, t2y, &temp2.y)
  LOG_ADD(LOG_FLOAT, t2z, &temp2.z)
  LOG_ADD(LOG_FLOAT, t2w, &temp2.w)
  LOG_ADD(LOG_FLOAT, bf, &battery_factor)
LOG_GROUP_STOP(ando_redundant)

PARAM_GROUP_START(ANDO_PARAM)
  PARAM_ADD(PARAM_FLOAT, cx, &_ge_gain.x)
  PARAM_ADD(PARAM_FLOAT, cy, &_ge_gain.y)
  PARAM_ADD(PARAM_FLOAT, cz, &_ge_gain.z)
  PARAM_ADD(PARAM_FLOAT, cw, &_ge_gain.w)
  PARAM_ADD(PARAM_FLOAT, mass, &MASS)
  PARAM_ADD(PARAM_UINT8, calc_ando, &ANDO_CALC)
  PARAM_ADD(PARAM_UINT8, enable_ando, &ANDO_ENABLE)
  PARAM_ADD(PARAM_FLOAT, ka, &MOTOR_KA)
  PARAM_ADD(PARAM_FLOAT, kb, &MOTOR_KB)
  PARAM_ADD(PARAM_FLOAT, ki, &BATTERY_KI)
  PARAM_ADD(PARAM_FLOAT, lpA, &LP_A)
  PARAM_ADD(PARAM_UINT8, enable_tau, &ANDO_TORQUE)
PARAM_GROUP_STOP(ANDO_PARAM)
