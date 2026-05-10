#ifndef PINOUT_H
#define PINOUT_H

#include "stm32l4xx_hal.h"

// USART2 / ST-LINK Virtual COM Port
#define UART_TX_GPIO_PORT      GPIOA
#define UART_TX_PIN            GPIO_PIN_2
#define UART_RX_GPIO_PORT      GPIOA
#define UART_RX_PIN            GPIO_PIN_3

// LED LD2 Nucleo
#define LED_GPIO_PORT          GPIOA
#define LED_PIN                GPIO_PIN_5

// PWM moteur : TIM2 CH1/CH2
#define MOTOR_LEFT_PWM_CHANNEL   TIM_CHANNEL_1  // PA0 / TIM2_CH1
#define MOTOR_RIGHT_PWM_CHANNEL  TIM_CHANNEL_2  // PA1 / TIM2_CH2

#define MOTOR_PWM_GPIO_PORT      GPIOA
#define MOTOR_LEFT_PWM_PIN       GPIO_PIN_0
#define MOTOR_RIGHT_PWM_PIN      GPIO_PIN_1

// Direction MotoDriver2 / L298N
#define MOTOR_IN1_GPIO_PORT      GPIOB
#define MOTOR_IN1_PIN            GPIO_PIN_0
#define MOTOR_IN2_GPIO_PORT      GPIOB
#define MOTOR_IN2_PIN            GPIO_PIN_1
#define MOTOR_IN3_GPIO_PORT      GPIOB
#define MOTOR_IN3_PIN            GPIO_PIN_2
#define MOTOR_IN4_GPIO_PORT      GPIOB
#define MOTOR_IN4_PIN            GPIO_PIN_10

// HC-SR04
#define HCSR04_TRIG_GPIO_PORT    GPIOC
#define HCSR04_TRIG_PIN          GPIO_PIN_7

#define HCSR04_ECHO_GPIO_PORT    GPIOB
#define HCSR04_ECHO_PIN          GPIO_PIN_6

#endif // PINOUT_H
