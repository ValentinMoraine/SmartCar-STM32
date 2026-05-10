#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD
} MotorDirection;

typedef struct {
    TIM_HandleTypeDef *pwmTimer;
    uint32_t leftChannel;
    uint32_t rightChannel;
    uint32_t pwmPeriod;
} MotorDriver;

void MotorDriver_Init(MotorDriver *driver,
                      TIM_HandleTypeDef *pwmTimer,
                      uint32_t leftChannel,
                      uint32_t rightChannel,
                      uint32_t pwmPeriod);

void MotorDriver_Stop(MotorDriver *driver);
void MotorDriver_SetLeft(MotorDriver *driver, MotorDirection direction, uint8_t speedPercent);
void MotorDriver_SetRight(MotorDriver *driver, MotorDirection direction, uint8_t speedPercent);

void MotorDriver_Forward(MotorDriver *driver, uint8_t speedPercent);
void MotorDriver_Backward(MotorDriver *driver, uint8_t speedPercent);
void MotorDriver_TurnLeft(MotorDriver *driver, uint8_t speedPercent);
void MotorDriver_TurnRight(MotorDriver *driver, uint8_t speedPercent);

#endif // MOTOR_DRIVER_H
