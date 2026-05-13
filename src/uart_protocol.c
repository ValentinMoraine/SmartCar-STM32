#include "uart_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_RX_IDLE_FRAME_MS 20U
#define UART_TX_SPIN_GUARD 1000000U

static void process_rx_char(UartProtocol *protocol, uint8_t c)
{
    if (c == '\n' || c == '\r' || c == '\0') {
        if ((protocol->lineIndex > 0U) && !protocol->pendingReady) {
            protocol->lineBuffer[protocol->lineIndex] = '\0';
            strncpy(protocol->pendingLine, protocol->lineBuffer, sizeof(protocol->pendingLine));
            protocol->pendingLine[sizeof(protocol->pendingLine) - 1U] = '\0';
            protocol->pendingReady = true;
        }

        protocol->lineIndex = 0U;
        memset(protocol->lineBuffer, 0, sizeof(protocol->lineBuffer));
    } else {
        if (protocol->lineIndex < (UART_RX_LINE_MAX - 1U)) {
            protocol->lineBuffer[protocol->lineIndex++] = (char)c;
            protocol->lastRxTickMs = HAL_GetTick();
        } else {
            protocol->lineIndex = 0U;
            memset(protocol->lineBuffer, 0, sizeof(protocol->lineBuffer));
            protocol->lastRxTickMs = 0U;
        }
    }
}

static void arm_rx_if_needed(UartProtocol *protocol)
{
    if (!protocol->rxRearmPending) {
        return;
    }

    const HAL_StatusTypeDef status = HAL_UART_Receive_IT(protocol->uart, &protocol->rxByte, 1U);
    if (status == HAL_OK) {
        protocol->rxRearmPending = false;
    }
}

static void uart_send_bytes(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t length)
{
    if (uart == NULL || data == NULL || length == 0U) {
        return;
    }

    for (uint16_t i = 0U; i < length; i++) {
        uint32_t guard = UART_TX_SPIN_GUARD;
        while (__HAL_UART_GET_FLAG(uart, UART_FLAG_TXE) == RESET) {
            if (guard-- == 0U) {
                return;
            }
        }

        uart->Instance->TDR = data[i];
    }

    uint32_t guard = UART_TX_SPIN_GUARD;
    while (__HAL_UART_GET_FLAG(uart, UART_FLAG_TC) == RESET) {
        if (guard-- == 0U) {
            return;
        }
    }
}

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
    protocol->pendingReady = false;
    protocol->lastRxTickMs = 0U;
    protocol->rxRearmPending = true;
    memset(protocol->lineBuffer, 0, sizeof(protocol->lineBuffer));
    memset(protocol->pendingLine, 0, sizeof(protocol->pendingLine));

    arm_rx_if_needed(protocol);
}

void UartProtocol_Task(UartProtocol *protocol)
{
    while (__HAL_UART_GET_FLAG(protocol->uart, UART_FLAG_RXNE) != RESET) {
        const uint8_t c = (uint8_t)(protocol->uart->Instance->RDR & 0xFFU);
        process_rx_char(protocol, c);
    }

    if (__HAL_UART_GET_FLAG(protocol->uart, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_FLAG(protocol->uart, UART_CLEAR_OREF);
    }

    arm_rx_if_needed(protocol);

    if (!protocol->pendingReady) {
        const uint32_t now = HAL_GetTick();

        __disable_irq();
        if (!protocol->pendingReady
            && (protocol->lineIndex > 0U)
            && ((now - protocol->lastRxTickMs) >= UART_RX_IDLE_FRAME_MS)) {
            protocol->lineBuffer[protocol->lineIndex] = '\0';
            strncpy(protocol->pendingLine, protocol->lineBuffer, sizeof(protocol->pendingLine));
            protocol->pendingLine[sizeof(protocol->pendingLine) - 1U] = '\0';

            protocol->lineIndex = 0U;
            memset(protocol->lineBuffer, 0, sizeof(protocol->lineBuffer));
            protocol->pendingReady = true;
        }
        __enable_irq();

        if (!protocol->pendingReady) {
            return;
        }
    }

    __disable_irq();
    char localLine[UART_RX_LINE_MAX];
    strncpy(localLine, protocol->pendingLine, sizeof(localLine));
    localLine[sizeof(localLine) - 1U] = '\0';

    protocol->pendingReady = false;
    memset(protocol->pendingLine, 0, sizeof(protocol->pendingLine));
    __enable_irq();

    handle_command(protocol, localLine);
}

void UartProtocol_RxCpltCallback(UartProtocol *protocol, UART_HandleTypeDef *uart)
{
    if (uart != protocol->uart) {
        return;
    }

    process_rx_char(protocol, protocol->rxByte);

    protocol->rxRearmPending = true;
    arm_rx_if_needed(protocol);
}

void UartProtocol_SendLine(UartProtocol *protocol, const char *line)
{
    char buffer[128];
    const int length = snprintf(buffer, sizeof(buffer), "%s\r\n", line);

    if (length > 0) {
        uint16_t txLength = (uint16_t)length;
        if ((size_t)length >= sizeof(buffer)) {
            txLength = (uint16_t)(sizeof(buffer) - 1U);
        }

        uart_send_bytes(protocol->uart, (const uint8_t *)buffer, txLength);
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
