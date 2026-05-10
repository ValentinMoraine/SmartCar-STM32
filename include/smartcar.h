#ifndef SMARTCAR_H
#define SMARTCAR_H

#include "hcsr04.h"
#include "motor_driver.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAR_STATE_INIT = 0,
    CAR_STATE_STOPPED,
    CAR_STATE_MANUAL,
    CAR_STATE_OBSTACLE,
    CAR_STATE_FAULT
} CarState;

typedef enum {
    CAR_MOVE_FWD = 0,
    CAR_MOVE_BACK,
    CAR_MOVE_LEFT,
    CAR_MOVE_RIGHT
} CarMoveDirection;

typedef struct {
    MotorDriver motors;
    Hcsr04 distanceSensor;

    CarState state;
    uint32_t lastDistanceCm;
    uint32_t obstacleThresholdCm;
    uint32_t lastMeasureTickMs;
    uint32_t measurePeriodMs;
    bool sensorOk;
} SmartCar;

void SmartCar_Init(SmartCar *car,
                   TIM_HandleTypeDef *pwmTimer,
                   uint32_t pwmPeriod,
                   TIM_HandleTypeDef *microTimer);

void SmartCar_Task(SmartCar *car);

const char *SmartCar_StateToString(CarState state);

bool SmartCar_Move(SmartCar *car, CarMoveDirection direction, uint8_t speedPercent);
void SmartCar_Stop(SmartCar *car);
void SmartCar_Reset(SmartCar *car);

bool SmartCar_ReadDistanceNow(SmartCar *car);
bool SmartCar_IsObstacleActive(const SmartCar *car);

#endif // SMARTCAR_H
