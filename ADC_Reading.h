#ifndef ADC_READING_H
#define ADC_READING_H

#include "stm32f4xx_hal.h"
#include "BT_DMA.h"        // لاستخدام هيكل BT_TelemetryData_t
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>        // لتعريف NULL

/* ===========================================================================
   بنية البيانات الفيزيائية (للمكتبات الخارجية)
   =========================================================================== */
typedef struct {
    /* تيارات المحرك (Amperes) */
    float u_current_raw;
    float v_current_raw;
    float w_current_raw;
    
    /* جهود المحرك (Volts) */
    float u_voltage_raw;
    float v_voltage_raw;
    float w_voltage_raw;
    float dc_bus_voltage_raw;
    
    /* درجة الحرارة (Celsius) */
    float temperature_raw;
    
    /* قيم خام من الـ DMA (للمكتبات المتقدمة) */
    uint16_t adc_raw[7];
} ADC_Data_t;

/* ===========================================================================
   واجهة برمجة المكتبة (Public API)
   =========================================================================== */
void ADC_Reading_Init(ADC_HandleTypeDef *hadc);
void ADC_Reading_Update(void);
const ADC_Data_t* ADC_Reading_GetData(void);
uint16_t ADC_Reading_GetRawValue(uint8_t channel_index);
void ADC_Reading_SendViaBluetooth(void);

#endif /* ADC_READING_H */