#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "smartcar.h"
#include "stm32l4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define UART_RX_LINE_MAX 96

typedef struct {
    UART_HandleTypeDef *uart;
    SmartCar *car;

    uint8_t rxByte;
    char lineBuffer[UART_RX_LINE_MAX];
    char pendingLine[UART_RX_LINE_MAX];
    volatile uint16_t lineIndex;
    volatile bool pendingReady;
    volatile uint32_t lastRxTickMs;
    volatile bool rxRearmPending;
} UartProtocol;

void UartProtocol_Init(UartProtocol *protocol,
                       UART_HandleTypeDef *uart,
                       SmartCar *car);

void UartProtocol_Task(UartProtocol *protocol);
void UartProtocol_RxCpltCallback(UartProtocol *protocol, UART_HandleTypeDef *uart);

void UartProtocol_SendLine(UartProtocol *protocol, const char *line);
void UartProtocol_SendStatus(UartProtocol *protocol);
void UartProtocol_SendDistance(UartProtocol *protocol);

#endif // UART_PROTOCOL_H
