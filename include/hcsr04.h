#ifndef HCSR04_H
#define HCSR04_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

typedef enum {
    HCSR04_OK = 0,
    HCSR04_TIMEOUT
} Hcsr04Status;

typedef struct {
    TIM_HandleTypeDef *microTimer;
    uint32_t timeoutUs;
} Hcsr04;

void Hcsr04_Init(Hcsr04 *sensor, TIM_HandleTypeDef *microTimer, uint32_t timeoutUs);
Hcsr04Status Hcsr04_ReadDistanceCm(Hcsr04 *sensor, uint32_t *distanceCm);

#endif // HCSR04_H
