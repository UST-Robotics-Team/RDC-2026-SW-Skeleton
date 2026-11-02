#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "main.h"
#include <stdbool.h>
typedef struct { uint16_t raw; uint8_t d0; } IR_Sample;
typedef struct { uint16_t distance_mm; uint8_t range_status; bool valid; } TOF_Sample;

HAL_StatusTypeDef IR_ReadDigital(uint8_t *d0);
HAL_StatusTypeDef IR_Read(IR_Sample *sample);

#endif
