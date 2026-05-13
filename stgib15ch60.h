/**
 ******************************************************************************
 * @file    stgib15ch60.h
 * @brief   STGIB15CH60 IPM Driver Library — STM32F411ECU (ZC323000 board)
 *
 * Handles:
 *   ✓ TIM1  3-phase complementary SPWM with 3rd-harmonic injection
 *   ✓ V/f   Linear voltage-frequency control with low-speed boost
 *   ✓ BKIN  Emergency-stop via PA12 (hardware break)
 *   ✓ Cin   Overcurrent/fault latch detection on PB2
 *   ✓ Dead-time  1 µs hardware dead-time (TIM1 BDTR)
 *   ✓ Ramp  Soft start and soft stop acceleration profile
 *
 * Stub hooks (weak — define in your application to override):
 *   • STGIB_Callback_OnFault()   — fault notification
 *   • STGIB_Callback_OnRunning() — reached target speed
 *   • STGIB_Callback_OnStopped() — fully stopped
 *
 * Integration hooks (call from other modules in future updates):
 *   • Display  : STGIB_Hook_GetCurrentHz / GetTargetHz / GetState / IsForward
 *   • Buttons  : STGIB_Hook_CmdStart / CmdStop / CmdSetHz / CmdToggleDir
 *   • ADC/NTC  : STGIB_Hook_ReportOvercurrent / Overvoltage / Overtemp
 *
 * ──────────────────────────────────────────────────────────────────
 *  Pin Assignments  (fixed by ZC323000 + STGIB15CH60 schematic)
 * ──────────────────────────────────────────────────────────────────
 *  PA8  = TIM1_CH1  AF1 → HIN(U)   high-side U gate (R1 1K series)
 *  PA9  = TIM1_CH2  AF1 → HIN(V)   high-side V gate (R2 1K series)
 *  PA10 = TIM1_CH3  AF1 → HIN(W)   high-side W gate (R3 1K series)
 *  PB13 = TIM1_CH1N AF1 → LIN(U)   low-side  U gate (R4 1K series)
 *  PB14 = TIM1_CH2N AF1 → LIN(V)   low-side  V gate (R5 1K series)
 *  PB15 = TIM1_CH3N AF1 → LIN(W)   low-side  W gate (R6 1K series)
 *  PA12 = TIM1_BKIN AF1 → EM_STOP  break input, active LOW (R52 0Ω)
 *  PB2  = GPIO_IN       → Cin      overcurrent/OD latch, active LOW
 * ──────────────────────────────────────────────────────────────────
 ******************************************************************************
 */

#ifndef STGIB15CH60_H
#define STGIB15CH60_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

/* ===========================================================================
 *  Compile-time configuration — change only these values if needed
 * =========================================================================== */

/** TIM1 input clock in Hz.
 *  STM32F411 default: sysclk = 84 MHz, APB2 = 84 MHz (prescaler = 1),
 *  therefore TIM1 clock = 84 MHz. Adjust if you use a different PLL config. */
#define STGIB_TIM_CLK_HZ        84000000UL

/** Switching (carrier) frequency in Hz.
 *  STGIB15CH60 is rated for up to 20 kHz.  10 kHz is a good trade-off. */
#define STGIB_CARRIER_HZ        10000U

/** Auto-reload register value — computed from above.
 *  Center-aligned (up-down) mode: F_sw = F_clk / (2 × ARR)
 *  → ARR = 84 000 000 / (2 × 10 000) = 4200 */
#define STGIB_ARR               (STGIB_TIM_CLK_HZ / (2U * STGIB_CARRIER_HZ))

/** Hardware dead-time counter value (BDTR DTG field).
 *  Formula (DTG < 128): dead_time = DTG / F_clk
 *  84 counts @ 84 MHz → 1.0 µs. Adequate for STGIB15CH60 IGBT turn-off. */
#define STGIB_DEAD_TIME_CNT     84U

/** Resolution of the pre-computed per-phase CCR sine table.
 *  1000 entries → ≤ 0.36 ° angular resolution.                           */
#define STGIB_SIN_TABLE_SIZE    1000U

/** Maximum allowed output frequency (Hz). */
#define STGIB_MAX_FREQ_HZ       60.0f

/** Minimum non-zero output frequency (Hz). */
#define STGIB_MIN_FREQ_HZ       1.0f

/** Base frequency for V/f law (Hz). Above this, V is constant.            */
#define STGIB_BASE_FREQ_HZ      50.0f

/** Low-speed voltage boost (fraction of V_max, 0 = no boost).
 *  Compensates stator resistance drop.  0.08 ≈ 8 % boost at 0 Hz.       */
#define STGIB_BOOST             0.08f

/** Acceleration / deceleration rate (Hz per second). */
#define STGIB_RAMP_RATE_HZ_S    20.0f

/* ===========================================================================
 *  Types
 * =========================================================================== */

/** Operating state of the driver */
typedef enum {
    STGIB_STATE_STOPPED  = 0,  /**< PWM disabled, motor at rest            */
    STGIB_STATE_STARTING,      /**< Ramping up to target frequency          */
    STGIB_STATE_RUNNING,       /**< At or near target frequency             */
    STGIB_STATE_STOPPING,      /**< Ramping down to zero                    */
    STGIB_STATE_FAULT          /**< Fault latched — call STGIB_ClearFault() */
} STGIB_State_t;

/** Rotation direction */
typedef enum {
    STGIB_DIR_FORWARD = 0,
    STGIB_DIR_REVERSE
} STGIB_Dir_t;

/** Fault reason bitmap (multiple faults can be active simultaneously) */
typedef enum {
    STGIB_FAULT_NONE        = 0x00,
    STGIB_FAULT_OVERCURRENT = 0x01,  /**< Cin / OD pin asserted            */
    STGIB_FAULT_EM_STOP     = 0x02,  /**< BKIN / EM_STOP asserted          */
    STGIB_FAULT_OVERTEMP    = 0x04,  /**< Reported by NTC monitor          */
    STGIB_FAULT_OVERVOLTAGE = 0x08   /**< Reported by DC-bus monitor       */
} STGIB_FaultFlags_t;

/** Public status snapshot — safe to read from main loop */
typedef struct {
    STGIB_State_t      state;
    STGIB_Dir_t        direction;
    float              current_hz;      /**< Actual output frequency (Hz)  */
    float              target_hz;       /**< Requested output frequency     */
    float              modulation;      /**< Current modulation index 0..1  */
    uint8_t            fault_flags;     /**< Bitmask of STGIB_FaultFlags_t */
    uint32_t           fault_count;     /**< Total faults since reset       */
} STGIB_Status_t;

/* ===========================================================================
 *  Core API
 * =========================================================================== */

/**
 * @brief  Initialise the STGIB15CH60 interface.
 *
 *         Configures:
 *           • GPIOA 8/9/10  — TIM1_CH1/2/3  (AF1, push-pull, 50 MHz)
 *           • GPIOB 13/14/15 — TIM1_CH1N/2N/3N (AF1, push-pull, 50 MHz)
 *           • GPIOA 12      — TIM1_BKIN     (AF1, input, pull-up)
 *           • GPIOB 2       — Cin input     (GPIO input, pull-up)
 *           • TIM1          — center-aligned PWM, 10 kHz, dead-time 1 µs
 *           • NVIC          — TIM1_UP and TIM1_BRK interrupts
 *           • Pre-computed sine tables (all three phases)
 *
 * @note   Does NOT enable PWM outputs — call STGIB_Start() when ready.
 * @note   Call once, before any other STGIB_xxx function.
 */
void STGIB_Init(void);

/**
 * @brief  Start the motor with soft ramp.
 * @param  target_hz  Target steady-state frequency [STGIB_MIN_FREQ_HZ..STGIB_MAX_FREQ_HZ]
 * @param  dir        Rotation direction
 * @note   Safe to call from stopped or fault-cleared state.
 *         Returns immediately; actual ramp happens inside the ISR.
 */
void STGIB_Start(float target_hz, STGIB_Dir_t dir);

/**
 * @brief  Stop the motor.
 * @param  ramp_down  true  → ramp frequency to zero (controlled decel)
 *                    false → immediately disable PWM outputs (coast/free-wheel)
 */
void STGIB_Stop(bool ramp_down);

/**
 * @brief  Change target frequency while running.
 * @param  hz  New target [STGIB_MIN_FREQ_HZ..STGIB_MAX_FREQ_HZ]
 *             The ISR ramps at STGIB_RAMP_RATE_HZ_S.
 */
void STGIB_SetFrequency(float hz);

/**
 * @brief  Change direction (only effective when motor is stopped).
 */
void STGIB_SetDirection(STGIB_Dir_t dir);

/**
 * @brief  Atomically read the current driver status.
 * @return Snapshot of STGIB_Status_t (safe to read from main loop).
 */
STGIB_Status_t STGIB_GetStatus(void);

/**
 * @brief  Quick fault check (no structure copy overhead).
 * @return true if any fault flag is active.
 */
bool STGIB_IsFault(void);

/**
 * @brief  Clear latched fault and allow restart.
 * @note   Only clears if the physical fault condition is no longer present.
 *         Checks Cin and EM_STOP pins before clearing.
 * @return true if fault cleared, false if condition still present.
 */
bool STGIB_ClearFault(void);

/* ===========================================================================
 *  ISR entry-points — call from your interrupt handlers
 * =========================================================================== */

/**
 * @brief  Must be called from TIM1_UP_TIM10_IRQHandler().
 *
 *         Updates CCR1/CCR2/CCR3 for SPWM output,
 *         advances phase accumulator,
 *         handles ramp-up / ramp-down logic.
 */
void STGIB_TIM1_UpdateISR(void);

/**
 * @brief  Must be called from TIM1_BRK_TIM9_IRQHandler().
 *
 *         Latches EM_STOP fault, disables outputs, calls OnFault callback.
 */
void STGIB_TIM1_BreakISR(void);

/* ===========================================================================
 *  Callbacks  (weak — override in your application)
 * =========================================================================== */

/**
 * @brief  Called when any fault is detected (ISR or Cin poll context).
 * @param  prev_state  State the driver was in before the fault.
 * @param  flags       STGIB_FaultFlags_t bitmask of active faults.
 */
__weak void STGIB_Callback_OnFault(STGIB_State_t prev_state,
                                    uint8_t flags);

/**
 * @brief  Called when the motor reaches its target frequency (running state).
 */
__weak void STGIB_Callback_OnRunning(void);

/**
 * @brief  Called when the motor comes to a complete stop.
 */
__weak void STGIB_Callback_OnStopped(void);

/* ===========================================================================
 *  Integration hooks
 *  ─────────────────────────────────────────────────────────────────────────
 *  These are the ONLY functions other modules need to call / read.
 *  Future modules (display, buttons, ADC) connect here and nowhere else.
 *  Do not add direct calls into this driver from other modules.
 * =========================================================================== */

/* ---- Getters (called by display / telemetry module) ---- */
float          STGIB_Hook_GetCurrentHz(void);
float          STGIB_Hook_GetTargetHz(void);
float          STGIB_Hook_GetModulation(void);
STGIB_State_t  STGIB_Hook_GetState(void);
bool           STGIB_Hook_IsForward(void);
uint8_t        STGIB_Hook_GetFaultFlags(void);

/* ---- Commands (called by button / UI module) ---- */
void           STGIB_Hook_CmdStart(float hz);
void           STGIB_Hook_CmdStop(void);
void           STGIB_Hook_CmdSetHz(float delta_hz);  /**< +/- increment */
void           STGIB_Hook_CmdToggleDir(void);

/* ---- Fault reports (called by ADC / NTC / protection module) ---- */
/**
 * @brief  Report an overcurrent event detected by external ADC measurement.
 *         Triggers fault state; same effect as hardware Cin assertion.
 */
void STGIB_Hook_ReportOvercurrent(void);

/**
 * @brief  Report a DC-bus overvoltage event (from ADC monitor).
 */
void STGIB_Hook_ReportOvervoltage(void);

/**
 * @brief  Report overtemperature (from NTC/TSO measurement).
 */
void STGIB_Hook_ReportOvertemp(void);

#ifdef __cplusplus
}
#endif

#endif /* STGIB15CH60_H */
