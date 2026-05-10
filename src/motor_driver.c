#include "motor_driver.h"
#include "pinout.h"

static uint8_t clamp_speed(uint8_t speedPercent)
{
    if (speedPercent > 100U) {
        return 100U;
    }

    return speedPercent;
}

static uint32_t percent_to_compare(const MotorDriver *driver, uint8_t speedPercent)
{
    const uint8_t clamped = clamp_speed(speedPercent);
    return ((driver->pwmPeriod + 1U) * clamped) / 100U;
}

void MotorDriver_Init(MotorDriver *driver,
                      TIM_HandleTypeDef *pwmTimer,
                      uint32_t leftChannel,
                      uint32_t rightChannel,
                      uint32_t pwmPeriod)
{
    driver->pwmTimer = pwmTimer;
    driver->leftChannel = leftChannel;
    driver->rightChannel = rightChannel;
    driver->pwmPeriod = pwmPeriod;

    HAL_TIM_PWM_Start(driver->pwmTimer, driver->leftChannel);
    HAL_TIM_PWM_Start(driver->pwmTimer, driver->rightChannel);

    MotorDriver_Stop(driver);
}

void MotorDriver_Stop(MotorDriver *driver)
{
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_PORT, MOTOR_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_PORT, MOTOR_IN4_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(driver->pwmTimer, driver->leftChannel, 0);
    __HAL_TIM_SET_COMPARE(driver->pwmTimer, driver->rightChannel, 0);
}

void MotorDriver_SetLeft(MotorDriver *driver, MotorDirection direction, uint8_t speedPercent)
{
    switch (direction) {
    case MOTOR_DIR_FORWARD:
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
        break;

    case MOTOR_DIR_BACKWARD:
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
        break;

    case MOTOR_DIR_STOP:
    default:
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
        speedPercent = 0;
        break;
    }

    __HAL_TIM_SET_COMPARE(driver->pwmTimer,
                          driver->leftChannel,
                          percent_to_compare(driver, speedPercent));
}

void MotorDriver_SetRight(MotorDriver *driver, MotorDirection direction, uint8_t speedPercent)
{
    switch (direction) {
    case MOTOR_DIR_FORWARD:
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_PORT, MOTOR_IN3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN4_GPIO_PORT, MOTOR_IN4_PIN, GPIO_PIN_RESET);
        break;

    case MOTOR_DIR_BACKWARD:
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_PORT, MOTOR_IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN4_GPIO_PORT, MOTOR_IN4_PIN, GPIO_PIN_SET);
        break;

    case MOTOR_DIR_STOP:
    default:
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_PORT, MOTOR_IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN4_GPIO_PORT, MOTOR_IN4_PIN, GPIO_PIN_RESET);
        speedPercent = 0;
        break;
    }

    __HAL_TIM_SET_COMPARE(driver->pwmTimer,
                          driver->rightChannel,
                          percent_to_compare(driver, speedPercent));
}

void MotorDriver_Forward(MotorDriver *driver, uint8_t speedPercent)
{
    MotorDriver_SetLeft(driver, MOTOR_DIR_FORWARD, speedPercent);
    MotorDriver_SetRight(driver, MOTOR_DIR_FORWARD, speedPercent);
}

void MotorDriver_Backward(MotorDriver *driver, uint8_t speedPercent)
{
    MotorDriver_SetLeft(driver, MOTOR_DIR_BACKWARD, speedPercent);
    MotorDriver_SetRight(driver, MOTOR_DIR_BACKWARD, speedPercent);
}

void MotorDriver_TurnLeft(MotorDriver *driver, uint8_t speedPercent)
{
    MotorDriver_SetLeft(driver, MOTOR_DIR_BACKWARD, speedPercent);
    MotorDriver_SetRight(driver, MOTOR_DIR_FORWARD, speedPercent);
}

void MotorDriver_TurnRight(MotorDriver *driver, uint8_t speedPercent)
{
    MotorDriver_SetLeft(driver, MOTOR_DIR_FORWARD, speedPercent);
    MotorDriver_SetRight(driver, MOTOR_DIR_BACKWARD, speedPercent);
}
