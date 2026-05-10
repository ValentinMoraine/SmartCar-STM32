#include "smartcar.h"
#include "pinout.h"

#define DEFAULT_OBSTACLE_THRESHOLD_CM 20U
#define DEFAULT_MEASURE_PERIOD_MS     150U
#define HCSR04_TIMEOUT_US             30000U

static bool forward_allowed(const SmartCar *car)
{
    if (!car->sensorOk) {
        return false;
    }

    return car->lastDistanceCm >= car->obstacleThresholdCm;
}

void SmartCar_Init(SmartCar *car,
                   TIM_HandleTypeDef *pwmTimer,
                   uint32_t pwmPeriod,
                   TIM_HandleTypeDef *microTimer)
{
    car->state = CAR_STATE_INIT;
    car->lastDistanceCm = 999U;
    car->obstacleThresholdCm = DEFAULT_OBSTACLE_THRESHOLD_CM;
    car->lastMeasureTickMs = 0U;
    car->measurePeriodMs = DEFAULT_MEASURE_PERIOD_MS;
    car->sensorOk = true;

    MotorDriver_Init(&car->motors,
                     pwmTimer,
                     MOTOR_LEFT_PWM_CHANNEL,
                     MOTOR_RIGHT_PWM_CHANNEL,
                     pwmPeriod);

    Hcsr04_Init(&car->distanceSensor, microTimer, HCSR04_TIMEOUT_US);

    MotorDriver_Stop(&car->motors);
    car->state = CAR_STATE_STOPPED;
}

void SmartCar_Task(SmartCar *car)
{
    const uint32_t now = HAL_GetTick();

    if ((now - car->lastMeasureTickMs) >= car->measurePeriodMs) {
        car->lastMeasureTickMs = now;

        if (!SmartCar_ReadDistanceNow(car)) {
            MotorDriver_Stop(&car->motors);
            car->state = CAR_STATE_FAULT;
            return;
        }

        if (SmartCar_IsObstacleActive(car)) {
            // Sécurité locale : la carte coupe les moteurs sans attendre Qt.
            MotorDriver_Stop(&car->motors);

            if (car->state == CAR_STATE_MANUAL) {
                car->state = CAR_STATE_OBSTACLE;
            }
        }
    }

    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}

const char *SmartCar_StateToString(CarState state)
{
    switch (state) {
    case CAR_STATE_INIT:
        return "INIT";

    case CAR_STATE_STOPPED:
        return "STOPPED";

    case CAR_STATE_MANUAL:
        return "MANUAL";

    case CAR_STATE_OBSTACLE:
        return "OBSTACLE";

    case CAR_STATE_FAULT:
        return "FAULT";

    default:
        return "UNKNOWN";
    }
}

bool SmartCar_Move(SmartCar *car, CarMoveDirection direction, uint8_t speedPercent)
{
    if (speedPercent > 100U) {
        return false;
    }

    if (car->state == CAR_STATE_FAULT) {
        MotorDriver_Stop(&car->motors);
        return false;
    }

    if (direction == CAR_MOVE_FWD && !forward_allowed(car)) {
        MotorDriver_Stop(&car->motors);
        car->state = CAR_STATE_OBSTACLE;
        return false;
    }

    switch (direction) {
    case CAR_MOVE_FWD:
        MotorDriver_Forward(&car->motors, speedPercent);
        break;

    case CAR_MOVE_BACK:
        MotorDriver_Backward(&car->motors, speedPercent);
        break;

    case CAR_MOVE_LEFT:
        MotorDriver_TurnLeft(&car->motors, speedPercent);
        break;

    case CAR_MOVE_RIGHT:
        MotorDriver_TurnRight(&car->motors, speedPercent);
        break;

    default:
        return false;
    }

    car->state = CAR_STATE_MANUAL;
    return true;
}

void SmartCar_Stop(SmartCar *car)
{
    MotorDriver_Stop(&car->motors);

    if (car->state != CAR_STATE_FAULT) {
        car->state = CAR_STATE_STOPPED;
    }
}

void SmartCar_Reset(SmartCar *car)
{
    MotorDriver_Stop(&car->motors);

    if (!SmartCar_ReadDistanceNow(car)) {
        car->state = CAR_STATE_FAULT;
        return;
    }

    if (SmartCar_IsObstacleActive(car)) {
        car->state = CAR_STATE_OBSTACLE;
        return;
    }

    car->state = CAR_STATE_STOPPED;
}

bool SmartCar_ReadDistanceNow(SmartCar *car)
{
    uint32_t distance = 0U;
    const Hcsr04Status status = Hcsr04_ReadDistanceCm(&car->distanceSensor, &distance);

    if (status != HCSR04_OK) {
        car->sensorOk = false;
        return false;
    }

    car->sensorOk = true;
    car->lastDistanceCm = distance;

    return true;
}

bool SmartCar_IsObstacleActive(const SmartCar *car)
{
    return car->sensorOk && (car->lastDistanceCm < car->obstacleThresholdCm);
}
