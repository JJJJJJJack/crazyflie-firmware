#ifndef __ANDO_H__
#define __ANDO_H__

#include "stabilizer_types.h"

typedef struct vec4_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
  float w;
} vec4;

void VecInit(vec4 *vec);
void VecMath(vec4 a, vec4 b, vec4 *result, int sign);
void ANDOInit(float update_period);
void ANDOReset();
void ANDO(const sensorData_t *sensors, const state_t *state, control_t *control, control_t * compensate_control);
#endif //__ANDO_H__
