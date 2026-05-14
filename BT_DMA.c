/* BT_DMA.c — compatible implementation (use huart1 by default) */

#include "BT_DMA.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/* Declare huart1 (CubeMX generated) */
extern UART_HandleTypeDef huart1;
static UART_HandleTypeDef *g_huart = NULL;

static uint8_t g_rx_byte;
static char g_line[BT_DMA_MAX_LINE_LEN];
static uint16_t g_line_idx = 0;

static BT_Command_t g_cmd_queue[BT_DMA_CMD_QUEUE_SIZE];
static volatile uint8_t g_cmd_head = 0;
static volatile uint8_t g_cmd_tail = 0;
static volatile uint8_t g_cmd_count = 0;

static void parse_line_to_command(const char *line, BT_Command_t *out);
static void enqueue_cmd_from_isr(const BT_Command_t *cmd)
{
    if (g_cmd_count < BT_DMA_CMD_QUEUE_SIZE) {
        g_cmd_queue[g_cmd_head] = *cmd;
        g_cmd_head = (g_cmd_head + 1) % BT_DMA_CMD_QUEUE_SIZE;
        g_cmd_count++;
    } else {
        g_cmd_tail = (g_cmd_tail + 1) % BT_DMA_CMD_QUEUE_SIZE;
        g_cmd_queue[g_cmd_head] = *cmd;
        g_cmd_head = (g_cmd_head + 1) % BT_DMA_CMD_QUEUE_SIZE;
    }
}

bool BT_DMA_GetCommand(BT_Command_t *out_cmd)
{
    if (g_cmd_count == 0) return false;
    __disable_irq();
    *out_cmd = g_cmd_queue[g_cmd_tail];
    g_cmd_tail = (g_cmd_tail + 1) % BT_DMA_CMD_QUEUE_SIZE;
    g_cmd_count--;
    __enable_irq();
    return true;
}

bool BT_DMA_InitWithHandle(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return false;
    g_huart = huart;
    __disable_irq();
    g_line_idx = 0;
    g_line[0] = '\0';
    g_cmd_head = g_cmd_tail = g_cmd_count = 0;
    __enable_irq();
    if (HAL_UART_Receive_IT(g_huart, &g_rx_byte, 1) != HAL_OK) return false;
    return true;
}

bool BT_DMA_Init(void)
{
    return BT_DMA_InitWithHandle(&huart1);
}

void BT_DMA_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != g_huart) return;
    uint8_t c = g_rx_byte;
    if (c == '\r') { }
    else if (c == '\n') {
        if (g_line_idx > 0) {
            g_line[g_line_idx] = '\0';
            BT_Command_t cmd;
            memset(&cmd, 0, sizeof(cmd));
            parse_line_to_command(g_line, &cmd);
            enqueue_cmd_from_isr(&cmd);
            g_line_idx = 0;
            g_line[0] = '\0';
        }
    } else {
        if (g_line_idx < (BT_DMA_MAX_LINE_LEN - 1)) {
            g_line[g_line_idx++] = (char)c;
        } else {
            g_line_idx = 0;
            g_line[0] = '\0';
        }
    }
    HAL_UART_Receive_IT(g_huart, &g_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) __attribute__((weak));
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { BT_DMA_RxCpltCallback(huart); }

void BT_DMA_TxCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) __attribute__((weak));
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { BT_DMA_TxCpltCallback(huart); }

static void parse_line_to_command(const char *line, BT_Command_t *out)
{
    if (!line || !out) return;
    memset(out, 0, sizeof(*out));
    while (*line == ' ' || *line == '\t') line++;
    strncpy(out->raw, line, BT_DMA_MAX_LINE_LEN - 1);
    out->raw[BT_DMA_MAX_LINE_LEN - 1] = '\0';

    char tmp[BT_DMA_MAX_LINE_LEN];
    size_t L = strlen(line);
    if (L >= sizeof(tmp)) L = sizeof(tmp) - 1;
    for (size_t i = 0; i < L; ++i) tmp[i] = (char)toupper((unsigned char)line[i]);
    tmp[L] = '\0';

    if (strcmp(tmp, "START") == 0) { out->type = BT_CMD_START; return; }
    if (strcmp(tmp, "STOP")  == 0) { out->type = BT_CMD_STOP;  return; }
    if (strcmp(tmp, "FWD")   == 0) { out->type = BT_CMD_DIRECTION_FWD; return; }
    if (strcmp(tmp, "REV")   == 0) { out->type = BT_CMD_DIRECTION_REV; return; }

    char key[16] = {0}; float val = 0.0f;
    if (sscanf(line, "%15[^=]=%f", key, &val) == 2) {
        for (char *p = key; *p; ++p) *p = (char)toupper((unsigned char)*p);
        if (strcmp(key, "FREQ") == 0) { out->type = BT_CMD_SET_FREQUENCY; out->param = val; return; }
        if (strcmp(key, "TRQ")  == 0) { out->type = BT_CMD_SET_TORQUE;    out->param = val; return; }
        if (strcmp(key, "SPD")  == 0) { out->type = BT_CMD_SET_SPEED;     out->param = val; return; }
        if (strcmp(key, "POL")  == 0 || strcmp(key, "POLES") == 0) {
            uint8_t p = (uint8_t)val;
            switch (p) { case 2: out->type = BT_CMD_POLES_2; out->poles = 2; break;
                         case 4: out->type = BT_CMD_POLES_4; out->poles = 4; break;
                         case 6: out->type = BT_CMD_POLES_6; out->poles = 6; break;
                         case 8: out->type = BT_CMD_POLES_8; out->poles = 8; break;
                         default: out->type = BT_CMD_RAW_LINE; break; }
            return;
        }
    }
    out->type = BT_CMD_RAW_LINE;
}

bool BT_DMA_SendTelemetry(const BT_TelemetryData_t *data)
{
    if (g_huart == NULL || data == NULL) return false;
    char txbuf[384];
    int len = snprintf(txbuf, sizeof(txbuf),
        "sp:%u\ntr:%.2f\nfr:%.1f\ndi:%u\npo:%u\n"
        "NT:%.1f\nvb:%.1f\nib:%.2f\n"
        "iu:%.2f\niv:%.2f\niw:%.2f\n"
        "vu:%u\nvv:%u\nvw:%u\nfl:%u\nbr:%u\n",
        (unsigned int)data->sp,
        data->tr,
        data->fr,
        (unsigned int)data->di,
        (unsigned int)data->po,
        data->NT,
        data->vb,
        data->ib,
        data->iu, data->iv, data->iw,
        (unsigned int)data->vu, (unsigned int)data->vv, (unsigned int)data->vw,
        (unsigned int)data->fl, (unsigned int)data->br
    );
    if (len <= 0 || (size_t)len >= sizeof(txbuf)) return false;
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(g_huart, (uint8_t*)txbuf, (uint16_t)len);
    if (st == HAL_OK) return true;
    st = HAL_UART_Transmit(g_huart, (uint8_t*)txbuf, (uint16_t)len, 100);
    return (st == HAL_OK);
}

/* Backward-compatible wrapper definition (one place only) */
bool BT_SendTelemetry(const BT_TelemetryData_t *data) { return BT_DMA_SendTelemetry(data); }