#include "hcsr04.h"
#include "pinout.h"

static uint32_t micros(Hcsr04 *sensor)
{
    return __HAL_TIM_GET_COUNTER(sensor->microTimer);
}

static uint32_t elapsed_us(uint32_t start, uint32_t now)
{
    return now - start; // Valable grâce au débordement naturel en uint32_t.
}

static void delay_us(Hcsr04 *sensor, uint32_t delay)
{
    const uint32_t start = micros(sensor);

    while (elapsed_us(start, micros(sensor)) < delay) {
        // Attente active courte, adaptée ici au TRIG de 10 µs.
    }
}

void Hcsr04_Init(Hcsr04 *sensor, TIM_HandleTypeDef *microTimer, uint32_t timeoutUs)
{
    sensor->microTimer = microTimer;
    sensor->timeoutUs = timeoutUs;

    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}

Hcsr04Status Hcsr04_ReadDistanceCm(Hcsr04 *sensor, uint32_t *distanceCm)
{
    uint32_t start = 0;
    uint32_t pulseStart = 0;
    uint32_t pulseDuration = 0;

    // Impulsion TRIG : au moins 10 µs.
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    delay_us(sensor, 2);

    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    delay_us(sensor, 10);

    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    // Attente front montant ECHO.
    start = micros(sensor);
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_RESET) {
        if (elapsed_us(start, micros(sensor)) > sensor->timeoutUs) {
            return HCSR04_TIMEOUT;
        }
    }

    pulseStart = micros(sensor);

    // Attente front descendant ECHO.
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET) {
        if (elapsed_us(pulseStart, micros(sensor)) > sensor->timeoutUs) {
            return HCSR04_TIMEOUT;
        }
    }

    pulseDuration = elapsed_us(pulseStart, micros(sensor));

    // Distance approximative en cm : durée_us / 58.
    // Aller-retour du son : formule classique pour HC-SR04.
    *distanceCm = pulseDuration / 58U;

    return HCSR04_OK;
}
