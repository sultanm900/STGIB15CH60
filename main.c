/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  @file           : main.c
  @brief          : Main program body
  ******************************************************************************
  @attention
  ******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BT_DMA.h"          /* ✅ تضمين مكتبة البلوتوث (يجب أن يكون بعد main.h) */
#include "ADC_Reading.h"     /* ✅ تضمين مكتبة قراءة المحسسات */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**
 * @brief هيكل حالة النظام
 */
typedef struct {
    bool   motor_running;   /* حالة التشغيل */
    bool   direction_fwd;   /* الاتجاه (true=أمامي) */
    uint8_t poles;          /* عدد الأقطاب */
    float  freq_ref;        /* التردد المرجعي */
    float  torque_ref;      /* العزم المرجعي */
    float  speed_ref;       /* السرعة المرجعية */
    uint8_t fault_flags;    /* أعلام الأعطال */
    uint8_t break_reason;   /* سبب التوقف */
    uint32_t last_telemetry_tick; /* توقيت آخر إرسال */
} SystemState_t;

static SystemState_t sys = {
    .motor_running       = false,
    .direction_fwd       = true,
    .poles               = 4,
    .freq_ref            = 50.0f,
    .torque_ref          = 0.0f,
    .speed_ref           = 0.0f,
    .fault_flags         = 0,
    .break_reason        = 0,
    .last_telemetry_tick = 0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Process_BT_Commands(void);
static void Update_Motor_Control(void);
static void Send_Telemetry_Periodic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief معالجة أوامر البلوتوث الواردة
 */
static void Process_BT_Commands(void)
{
    BT_Command_t cmd;
    
    /* سحب جميع الأوامر المتاحة من الطابور */
    while (BT_DMA_GetCommand(&cmd)) {
        switch (cmd.type) {
            case BT_CMD_START:
                sys.motor_running = true;
                sys.break_reason  = 0;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
                break;
                
            case BT_CMD_STOP:
                sys.motor_running = false;
                sys.break_reason  = 2;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
                break;
                
            case BT_CMD_DIRECTION_FWD:
                sys.direction_fwd = true;
                break;
                
            case BT_CMD_DIRECTION_REV:
                sys.direction_fwd = false;
                break;
                
            case BT_CMD_POLES_2: case BT_CMD_POLES_4:
            case BT_CMD_POLES_6: case BT_CMD_POLES_8:
                sys.poles = cmd.poles;
                break;
                
            case BT_CMD_SET_FREQUENCY:
                sys.freq_ref = cmd.param;
                if (sys.freq_ref < 0.0f) sys.freq_ref = 0.0f;
                if (sys.freq_ref > 100.0f) sys.freq_ref = 100.0f;
                break;
                
            case BT_CMD_SET_TORQUE:
                sys.torque_ref = cmd.param;
                if (sys.torque_ref < 0.0f) sys.torque_ref = 0.0f;
                if (sys.torque_ref > 2.0f) sys.torque_ref = 2.0f;
                break;
                
            case BT_CMD_SET_SPEED:
                sys.speed_ref = cmd.param;
                if (sys.speed_ref < 0.0f) sys.speed_ref = 0.0f;
                if (sys.speed_ref > 3000.0f) sys.speed_ref = 3000.0f;
                break;
                
            default:
                break;
        }
    }
}

/**
 * @brief حلقة التحكم الرئيسية وتحديث متغيرات المحرك
 */
static void Update_Motor_Control(void)
{
    /* جلب أحدث قراءات المحسسات من المكتبة */
    const ADC_Data_t *adc_data = ADC_Reading_GetData();
    
    /* هنا يتم تطبيق خوارزمية التحكم (FOC/SVPWM) مستقبلاً */
    /* مثال: FOC_Update(sys.freq_ref, sys.torque_ref, adc_data); */
    
    if (!sys.motor_running) {
        /* منطق التوقف الآمن */
    }
}

/**
 * @brief إرسال بيانات القياس عن بعد للتطبيق بشكل دوري
 */
static void Send_Telemetry_Periodic(void)
{
    uint32_t now = HAL_GetTick();
    
    /* إرسال البيانات كل 100 مللي ثانية */
    if ((now - sys.last_telemetry_tick) >= 100) {
        const ADC_Data_t *adc_data = ADC_Reading_GetData();
        
        /* ✅ استخدام الهيكل الصحيح كما هو معرف في BT_DMA.h */
        BT_TelemetryData_t telemetry = {0};
        
        uint16_t sync_speed = (uint16_t)(120.0f * sys.freq_ref / sys.poles);
        
        telemetry.speed_rpm         = sys.motor_running ? sync_speed : 0;
        telemetry.torque_ref        = sys.torque_ref;
        telemetry.frequency_hz      = sys.freq_ref;
        telemetry.direction         = sys.motor_running ? (sys.direction_fwd ? 1 : 2) : 0;
        telemetry.power_factor_x10  = 8;
        telemetry.temperature_c     = adc_data->temperature_raw;
        telemetry.bus_voltage       = adc_data->dc_bus_voltage_raw;
        telemetry.bus_current       = 0.0f; /* يحتاج قناة إضافية */
        telemetry.phase_u_current   = adc_data->u_current_raw;
        telemetry.phase_v_current   = adc_data->v_current_raw;
        telemetry.phase_w_current   = adc_data->w_current_raw;
        telemetry.phase_u_voltage   = (uint16_t)adc_data->u_voltage_raw;
        telemetry.phase_v_voltage   = (uint16_t)adc_data->v_voltage_raw;
        telemetry.phase_w_voltage   = (uint16_t)adc_data->w_voltage_raw;
        telemetry.fault_flags       = sys.fault_flags;
        telemetry.break_reason      = sys.break_reason;
        
        /* ✅ استخدام الدالة الصحيحة */
        BT_DMA_SendTelemetry(&telemetry);
        
        sys.last_telemetry_tick = now;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  /* 1. تهيئة مكتبة البلوتوث */
  if (!BT_DMA_Init()) {
      Error_Handler();
  }
  
  /* 2. تهيئة مكتبة قراءة ADC */
  ADC_Reading_Init(&hadc1);
  
  /* 3. بدء التحويل المستمر عبر DMA */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)hadc1.DMA_Handle->Instance->M0AR, 7);
  
  /* 4. بدء مؤقت TIM10 */
  HAL_TIM_Base_Start_IT(&htim10);
  
  /* تهيئة التوقيت */
  sys.last_telemetry_tick = HAL_GetTick();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* 1. معالجة أوامر البلوتوث */
    Process_BT_Commands();
    
    /* 2. تحديث التحكم (كل 10ms) */
    static uint32_t last_ctrl = 0;
    if ((HAL_GetTick() - last_ctrl) >= 10) {
        ADC_Reading_Update();           /* تحديث التحويلات */
        Update_Motor_Control();         /* تطبيق المنطق */
        last_ctrl = HAL_GetTick();
    }
    
    /* 3. إرسال البيانات */
    Send_Telemetry_Periodic();
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Timer Period Elapsed Callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        /* يمكن استدعاء Update_Motor_Control() هنا لضمان توقيت دقيق */
    }
}

/**
 * @brief ADC Conversion Complete Callback
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        /* التحديث يتم عبر DMA، لكن يمكن إضافة منطق طارئ هنا */
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
