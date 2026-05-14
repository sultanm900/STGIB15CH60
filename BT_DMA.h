#ifndef BT_DMA_H
#define BT_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef BT_DMA_MAX_LINE_LEN
#define BT_DMA_MAX_LINE_LEN   64U
#endif

#ifndef BT_DMA_CMD_QUEUE_SIZE
#define BT_DMA_CMD_QUEUE_SIZE 8U
#endif

typedef struct {
    uint16_t sp;
    float    tr;
    float    fr;
    uint8_t  di;
    uint8_t  po;
    float    NT;
    float    vb;
    float    ib;
    float    iu, iv, iw;
    uint16_t vu, vv, vw;
    uint8_t  fl;
    uint8_t  br;

    /* Backwards-compatible duplicate fields (optional) */
    uint16_t speed_rpm;
    float    torque_ref;
    float    frequency_hz;
    uint8_t  direction;
    uint8_t  power_factor_x10;
    float    temperature_c;
    float    bus_voltage;
    float    bus_current;
    float    phase_u_current;
    float    phase_v_current;
    float    phase_w_current;
    uint16_t phase_u_voltage;
    uint16_t phase_v_voltage;
    uint16_t phase_w_voltage;
    uint8_t  fault_flags;
    uint8_t  break_reason;
    uint8_t  poles;
} BT_TelemetryData_t;

typedef BT_TelemetryData_t BT_Telemetry_t;

typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_START,
    BT_CMD_STOP,
    BT_CMD_DIRECTION_FWD,
    BT_CMD_DIRECTION_REV,
    BT_CMD_POLES_2,
    BT_CMD_POLES_4,
    BT_CMD_POLES_6,
    BT_CMD_POLES_8,
    BT_CMD_SET_FREQUENCY,
    BT_CMD_SET_TORQUE,
    BT_CMD_SET_SPEED,
    BT_CMD_RAW_LINE
} BT_CommandType_t;

typedef struct {
    BT_CommandType_t type;
    union {
        float param;
        uint8_t poles;
    };
    char raw[BT_DMA_MAX_LINE_LEN];
} BT_Command_t;

/* Public API */
bool BT_DMA_Init(void);                            /* uses extern huart1 by default */
bool BT_DMA_InitWithHandle(UART_HandleTypeDef *huart);
bool BT_DMA_GetCommand(BT_Command_t *out_cmd);
bool BT_DMA_SendTelemetry(const BT_TelemetryData_t *data);
void BT_DMA_RxCpltCallback(UART_HandleTypeDef *huart);
void BT_DMA_TxCpltCallback(UART_HandleTypeDef *huart);

/* Backward-compatible wrapper (declaration only) */
bool BT_SendTelemetry(const BT_TelemetryData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* BT_DMA_H */