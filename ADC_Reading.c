/**
  @file    ADC_Reading.c
  @brief   مكتبة قراءة المحسسات عبر ADC + DMA لـ STM32F411
*/

#include "ADC_Reading.h"
#include <math.h>
#include <string.h>

/* ===========================================================================
   ثوابت المعايرة
   =========================================================================== */
#define VREF_VOLTAGE           3.3f
#define ADC_MAX_RES            4095.0f
#define VOLT_DIV_RATIO         (10.0f / (680.0f + 10.0f))
#define CURRENT_AMPLIFIER_GAIN 8.0f
#define SHUNT_RESISTOR         0.02f

/* ===========================================================================
   متغيرات داخلية
   =========================================================================== */
static ADC_HandleTypeDef *p_hadc = NULL;
static uint16_t *p_dma_buffer = NULL;
static ADC_Data_t current_data = {0};

/* ===========================================================================
   دوال مساعدة
   =========================================================================== */
static float convert_to_voltage(uint16_t raw_adc)
{
    float v_sense = (float)raw_adc / ADC_MAX_RES * VREF_VOLTAGE;
    return v_sense / VOLT_DIV_RATIO;
}

static float convert_to_current(uint16_t raw_adc)
{
    float v_sense = (float)raw_adc / ADC_MAX_RES * VREF_VOLTAGE;
    return v_sense / (CURRENT_AMPLIFIER_GAIN * SHUNT_RESISTOR);
}

static float convert_to_temp(uint16_t raw_adc)
{
    float voltage = (float)raw_adc / ADC_MAX_RES * VREF_VOLTAGE;
    float r_ntc = 10000.0f * (3.3f - voltage) / voltage;
    float temp_kelvin = 1.0f / (1.0f/298.15f + (1.0f/3950.0f) * logf(r_ntc / 10000.0f));
    return temp_kelvin - 273.15f;
}

/* ===========================================================================
   التهيئة
   =========================================================================== */
void ADC_Reading_Init(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL) return;
    p_hadc = hadc;
    p_dma_buffer = (uint16_t*)hadc->DMA_Handle->Instance->M0AR;
    memset((void*)&current_data, 0, sizeof(ADC_Data_t));
}

/* ===========================================================================
   التحديث الرئيسي
   =========================================================================== */
void ADC_Reading_Update(void)
{
    if (p_dma_buffer == NULL) return;

    /* الترتيب وفق adc.c:
       Rank 1: CH0 (PA0) -> Temp
       Rank 2: CH1 (PA1) -> U_Current
       Rank 3: CH2 (PA2) -> V_Current
       Rank 4: CH3 (PA3) -> W_Current
       Rank 5: CH4 (PA4) -> U_Voltage
       Rank 6: CH5 (PA5) -> V_Voltage
       Rank 7: CH6 (PA6) -> W_Voltage
    */
    current_data.adc_raw[0] = p_dma_buffer[0];  // Temp
    current_data.adc_raw[1] = p_dma_buffer[1];  // U_Current
    current_data.adc_raw[2] = p_dma_buffer[2];  // V_Current
    current_data.adc_raw[3] = p_dma_buffer[3];  // W_Current
    current_data.adc_raw[4] = p_dma_buffer[4];  // U_Voltage
    current_data.adc_raw[5] = p_dma_buffer[5];  // V_Voltage
    current_data.adc_raw[6] = p_dma_buffer[6];  // W_Voltage

    current_data.u_current_raw = convert_to_current(current_data.adc_raw[1]);
    current_data.v_current_raw = convert_to_current(current_data.adc_raw[2]);
    current_data.w_current_raw = convert_to_current(current_data.adc_raw[3]);

    current_data.u_voltage_raw = convert_to_voltage(current_data.adc_raw[4]);
    current_data.v_voltage_raw = convert_to_voltage(current_data.adc_raw[5]);
    current_data.w_voltage_raw = convert_to_voltage(current_data.adc_raw[6]);

    current_data.dc_bus_voltage_raw = 0.0f;
    current_data.temperature_raw = convert_to_temp(current_data.adc_raw[0]);
}

const ADC_Data_t* ADC_Reading_GetData(void)
{
    return &current_data;
}

uint16_t ADC_Reading_GetRawValue(uint8_t channel_index)
{
    if (channel_index >= 7) return 0;
    return current_data.adc_raw[channel_index];
}

/* ===========================================================================
   الربط مع مكتبة البلوتوث - ✅ مصحح وفق الحقول الصحيحة في BT_Telemetry_t
   =========================================================================== */
void ADC_Reading_SendViaBluetooth(void)
{
    /* ✅ استخدام الهيكل الصحيح */
    BT_Telemetry_t telemetry = {0};

    /* ✅ تعبئة البيانات باستخدام الأسماء الصحيحة للحقول */
    telemetry.iu = current_data.u_current_raw;
    telemetry.iv = current_data.v_current_raw;
    telemetry.iw = current_data.w_current_raw;
    
    telemetry.vu = (uint16_t)current_data.u_voltage_raw;
    telemetry.vv = (uint16_t)current_data.v_voltage_raw;
    telemetry.vw = (uint16_t)current_data.w_voltage_raw;
    
    telemetry.vb = current_data.dc_bus_voltage_raw;
    telemetry.ib = 0.0f;
    telemetry.NT = current_data.temperature_raw;

    /* حقول إضافية */
    telemetry.sp = 0;
    telemetry.tr = 0.0f;
    telemetry.fr = 0.0f;
    telemetry.di = 0;
    telemetry.po = 0;
    telemetry.fl = 0;
    telemetry.br = 0;

    /* ✅ استخدام الدالة الصحيحة */
    BT_SendTelemetry(&telemetry);
}