#include "uart_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool parse_speed(const char *text, uint8_t *speed)
{
    char *end = NULL;
    const long value = strtol(text, &end, 10);

    if (end == text || *end != '\0' || value < 0 || value > 100) {
        return false;
    }

    *speed = (uint8_t)value;
    return true;
}

static void handle_move(UartProtocol *protocol, const char *payload)
{
    // Format attendu : MOVE:FWD:40
    char direction[12] = {0};
    char speedText[8] = {0};
    uint8_t speed = 0;

    const int count = sscanf(payload, "%11[^:]:%7s", direction, speedText);
    if (count != 2) {
        UartProtocol_SendLine(protocol, "ERROR:INVALID_COMMAND");
        return;
    }

    if (!parse_speed(speedText, &speed)) {
        UartProtocol_SendLine(protocol, "ERROR:INVALID_SPEED");
        return;
    }

    CarMoveDirection moveDirection;

    if (strcmp(direction, "FWD") == 0) {
        moveDirection = CAR_MOVE_FWD;
    } else if (strcmp(direction, "BACK") == 0) {
        moveDirection = CAR_MOVE_BACK;
    } else if (strcmp(direction, "LEFT") == 0) {
        moveDirection = CAR_MOVE_LEFT;
    } else if (strcmp(direction, "RIGHT") == 0) {
        moveDirection = CAR_MOVE_RIGHT;
    } else {
        UartProtocol_SendLine(protocol, "ERROR:INVALID_COMMAND");
        return;
    }

    const bool ok = SmartCar_Move(protocol->car, moveDirection, speed);
    if (!ok) {
        if (protocol->car->state == CAR_STATE_OBSTACLE) {
            UartProtocol_SendLine(protocol, "ERROR:OBSTACLE_TOO_CLOSE");
        } else {
            UartProtocol_SendLine(protocol, "ERROR:MOVE_REFUSED");
        }

        UartProtocol_SendStatus(protocol);
        UartProtocol_SendDistance(protocol);
        return;
    }

    UartProtocol_SendLine(protocol, "OK");
    UartProtocol_SendStatus(protocol);
}

static void handle_command(UartProtocol *protocol, const char *line)
{
    if (strcmp(line, "PING") == 0) {
        UartProtocol_SendLine(protocol, "PONG");
        return;
    }

    if (strcmp(line, "STOP") == 0) {
        SmartCar_Stop(protocol->car);
        UartProtocol_SendLine(protocol, "OK");
        UartProtocol_SendStatus(protocol);
        return;
    }

    if (strcmp(line, "RESET") == 0) {
        SmartCar_Reset(protocol->car);
        UartProtocol_SendLine(protocol, "OK");
        UartProtocol_SendStatus(protocol);
        UartProtocol_SendDistance(protocol);
        return;
    }

    if (strcmp(line, "GET_STATUS") == 0) {
        UartProtocol_SendStatus(protocol);
        return;
    }

    if (strcmp(line, "GET_DISTANCE") == 0) {
        if (!SmartCar_ReadDistanceNow(protocol->car)) {
            UartProtocol_SendLine(protocol, "ERROR:SENSOR_TIMEOUT");
            UartProtocol_SendStatus(protocol);
            return;
        }

        UartProtocol_SendDistance(protocol);
        return;
    }

    if (strncmp(line, "MOVE:", 5) == 0) {
        handle_move(protocol, line + 5);
        return;
    }

    UartProtocol_SendLine(protocol, "ERROR:INVALID_COMMAND");
}

void UartProtocol_Init(UartProtocol *protocol,
                       UART_HandleTypeDef *uart,
                       SmartCar *car)
{
    protocol->uart = uart;
    protocol->car = car;
    protocol->rxByte = 0U;
    protocol->lineIndex = 0U;
    protocol->lineReady = false;
    memset(protocol->lineBuffer, 0, sizeof(protocol->lineBuffer));

    HAL_UART_Receive_IT(protocol->uart, &protocol->rxByte, 1U);
}

void UartProtocol_Task(UartProtocol *protocol)
{
    if (!protocol->lineReady) {
        return;
    }

    __disable_irq();
    char localLine[UART_RX_LINE_MAX];
    strncpy(localLine, protocol->lineBuffer, sizeof(localLine));
    localLine[sizeof(localLine) - 1U] = '\0';

    protocol->lineIndex = 0U;
    protocol->lineReady = false;
    memset(protocol->lineBuffer, 0, sizeof(protocol->lineBuffer));
    __enable_irq();

    handle_command(protocol, localLine);
}

void UartProtocol_RxCpltCallback(UartProtocol *protocol, UART_HandleTypeDef *uart)
{
    if (uart != protocol->uart) {
        return;
    }

    const uint8_t c = protocol->rxByte;

    if (c == '\n' || c == '\r') {
        if (protocol->lineIndex > 0U) {
            protocol->lineBuffer[protocol->lineIndex] = '\0';
            protocol->lineReady = true;
        }
    } else if (!protocol->lineReady) {
        if (protocol->lineIndex < (UART_RX_LINE_MAX - 1U)) {
            protocol->lineBuffer[protocol->lineIndex++] = (char)c;
        } else {
            protocol->lineIndex = 0U;
            protocol->lineReady = false;
        }
    }

    HAL_UART_Receive_IT(protocol->uart, &protocol->rxByte, 1U);
}

void UartProtocol_SendLine(UartProtocol *protocol, const char *line)
{
    char buffer[128];
    const int length = snprintf(buffer, sizeof(buffer), "%s\r\n", line);

    if (length > 0) {
        HAL_UART_Transmit(protocol->uart, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
    }
}

void UartProtocol_SendStatus(UartProtocol *protocol)
{
    char buffer[48];
    snprintf(buffer, sizeof(buffer), "STATE:%s", SmartCar_StateToString(protocol->car->state));
    UartProtocol_SendLine(protocol, buffer);
}

void UartProtocol_SendDistance(UartProtocol *protocol)
{
    char buffer[48];
    snprintf(buffer, sizeof(buffer), "DIST:%lu", (unsigned long)protocol->car->lastDistanceCm);
    UartProtocol_SendLine(protocol, buffer);
}
