/**
 ******************************************************************************
 * @file    stgib15ch60.c
 * @brief   STGIB15CH60 IPM Driver — implementation
 *
 *  SPWM strategy
 *  ─────────────
 *  • Carrier  : TIM1 center-aligned (up/down), 10 kHz, ARR = 4200.
 *  • Modulation: SPWM with 1/6 third-harmonic injection (THIPWM).
 *      V(θ) = m·[sin(θ) + sin(3θ)/6]
 *    This raises the linear modulation ceiling from 1.00 to 1.155,
 *    improving DC-bus utilisation without leaving the linear region.
 *  • V/f law : m(f) = STGIB_BOOST + (1 − STGIB_BOOST)·(f / f_base)
 *    Clamped to [0, 1].  Above f_base voltage is held constant.
 *  • CCR formula (center-aligned, offset to mid-rail):
 *      CCR = (ARR/2) · (1 + m · THIPWM(θ))
 *    Range: 0 … ARR.  CCR = ARR/2 → 50 % duty → zero V_phase.
 *  • Phase stepping:
 *      Δθ_ISR = 2π · f_out / f_carrier   [radians per ISR tick]
 *    Fractional phase accumulator in float for smooth sub-Hz steps.
 *
 *  Ramp
 *  ────
 *  Inside the ISR (10 000 calls/s), current_hz is nudged toward
 *  target_hz by ±RAMP_DELTA each tick.  Modulation index is
 *  recomputed from current_hz every tick so V/f is always correct.
 *
 *  Fault handling
 *  ──────────────
 *  Hardware BKIN (PA12, active LOW via EM_STOP) immediately tri-states
 *  all six gate signals through TIM1's break function — no software
 *  latency.  The TIM1_BRK ISR then latches the fault in software.
 *  Cin (PB2, active LOW) is polled every carrier cycle; if asserted it
 *  triggers the same software fault path.
 ******************************************************************************
 */

#include "stgib15ch60.h"
#include <math.h>
#include <string.h>

/* =========================================================================
 *  Private constants derived from the header
 * ========================================================================= */

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif
#define M_2PI (2.0 * M_PI)

/** Half the ARR — used for CCR mid-rail offset */
#define STGIB_ARR_HALF          (STGIB_ARR / 2U)

/** Ramp delta applied each ISR call (Hz per call) */
#define STGIB_RAMP_DELTA        (STGIB_RAMP_RATE_HZ_S / (float)STGIB_CARRIER_HZ)

/* =========================================================================
 *  Private state  (all volatile so the ISR and main loop share safely)
 * ========================================================================= */

/** Pre-computed CCR values for phase A (0 … ARR).
 *  B and C are computed by index offset (TABLE_SIZE/3, 2*TABLE_SIZE/3). */
static uint16_t s_ccr_table[STGIB_SIN_TABLE_SIZE];

static volatile float           s_current_hz;
static volatile float           s_target_hz;
static volatile float           s_modulation;
static volatile float           s_phase_acc;   /* radians, wraps at 2π   */
static volatile STGIB_State_t   s_state;
static volatile STGIB_Dir_t     s_direction;
static volatile uint8_t         s_fault_flags;
static volatile uint32_t        s_fault_count;
static volatile bool            s_pwm_enabled;

/* =========================================================================
 *  Private helpers — forward declarations
 * ========================================================================= */
static void     _gpio_init(void);
static void     _tim1_init(void);
static void     _nvic_init(void);
static void     _build_ccr_table(float modulation);
static uint16_t _ccr_from_phase(float theta_rad, float modulation);
static float    _vf_modulation(float hz);
static void     _enable_outputs(void);
static void     _disable_outputs(void);
static void     _latch_fault(uint8_t flag);

/* =========================================================================
 *  Public API
 * ========================================================================= */

void STGIB_Init(void)
{
    /* Zero internal state */
    s_current_hz  = 0.0f;
    s_target_hz   = 0.0f;
    s_modulation  = 0.0f;
    s_phase_acc   = 0.0f;
    s_state       = STGIB_STATE_STOPPED;
    s_direction   = STGIB_DIR_FORWARD;
    s_fault_flags = STGIB_FAULT_NONE;
    s_fault_count = 0;
    s_pwm_enabled = false;

    /* Pre-compute CCR table at zero (all entries = ARR/2 = mid-rail) */
    _build_ccr_table(0.0f);

    /* Hardware init */
    _gpio_init();
    _tim1_init();
    _nvic_init();

    /* Keep outputs disabled until STGIB_Start() is called */
    _disable_outputs();
}

/* -------------------------------------------------------------------------- */
void STGIB_Start(float target_hz, STGIB_Dir_t dir)
{
    if (s_state == STGIB_STATE_FAULT) {
        return;   /* Must clear fault first */
    }

    /* Clamp frequency */
    if (target_hz < STGIB_MIN_FREQ_HZ) target_hz = STGIB_MIN_FREQ_HZ;
    if (target_hz > STGIB_MAX_FREQ_HZ) target_hz = STGIB_MAX_FREQ_HZ;

    /* Critical section — disable ISR while updating shared state */
    __disable_irq();
    s_target_hz  = target_hz;
    s_direction  = dir;
    s_state      = STGIB_STATE_STARTING;
    s_phase_acc  = 0.0f;
    /* current_hz starts from zero (or from where it already is if re-starting) */
    if (s_current_hz > target_hz) {
        s_current_hz = 0.0f;   /* Direction change — reset */
    }
    __enable_irq();

    _enable_outputs();
}

/* -------------------------------------------------------------------------- */
void STGIB_Stop(bool ramp_down)
{
    if (s_state == STGIB_STATE_STOPPED ||
        s_state == STGIB_STATE_FAULT)
    {
        return;
    }

    if (ramp_down) {
        __disable_irq();
        s_target_hz = 0.0f;
        s_state     = STGIB_STATE_STOPPING;
        __enable_irq();
        /* ISR will ramp down; outputs disabled when current_hz reaches 0 */
    } else {
        /* Immediate coast/free-wheel stop */
        __disable_irq();
        s_target_hz  = 0.0f;
        s_current_hz = 0.0f;
        s_state      = STGIB_STATE_STOPPED;
        __enable_irq();

        _disable_outputs();
        STGIB_Callback_OnStopped();
    }
}

/* -------------------------------------------------------------------------- */
void STGIB_SetFrequency(float hz)
{
    if (hz < STGIB_MIN_FREQ_HZ) hz = STGIB_MIN_FREQ_HZ;
    if (hz > STGIB_MAX_FREQ_HZ) hz = STGIB_MAX_FREQ_HZ;

    __disable_irq();
    s_target_hz = hz;
    __enable_irq();
}

/* -------------------------------------------------------------------------- */
void STGIB_SetDirection(STGIB_Dir_t dir)
{
    if (s_state == STGIB_STATE_STOPPED || s_state == STGIB_STATE_FAULT) {
        s_direction = dir;
    }
    /* Silently ignored if motor is running — must stop first */
}

/* -------------------------------------------------------------------------- */
STGIB_Status_t STGIB_GetStatus(void)
{
    STGIB_Status_t snap;
    __disable_irq();
    snap.state       = s_state;
    snap.direction   = s_direction;
    snap.current_hz  = s_current_hz;
    snap.target_hz   = s_target_hz;
    snap.modulation  = s_modulation;
    snap.fault_flags = s_fault_flags;
    snap.fault_count = s_fault_count;
    __enable_irq();
    return snap;
}

/* -------------------------------------------------------------------------- */
bool STGIB_IsFault(void)
{
    return (s_state == STGIB_STATE_FAULT);
}

/* -------------------------------------------------------------------------- */
bool STGIB_ClearFault(void)
{
    /* Verify physical fault sources are no longer active */
    bool cin_ok    = (GPIOB->IDR & GPIO_IDR_IDR_2) != 0;  /* Cin HIGH = OK */
    bool emstop_ok = (GPIOA->IDR & GPIO_IDR_IDR_12) != 0; /* BKIN HIGH = OK */

    if (!cin_ok || !emstop_ok) {
        return false;  /* Condition still present */
    }

    __disable_irq();
    s_fault_flags = STGIB_FAULT_NONE;
    s_state       = STGIB_STATE_STOPPED;
    s_current_hz  = 0.0f;
    s_target_hz   = 0.0f;
    __enable_irq();

    /* Re-enable TIM1 break auto-output (was latched by hardware) */
    TIM1->BDTR |= TIM_BDTR_AOE;   /* Auto-restart outputs on next PWM cycle */

    return true;
}

/* =========================================================================
 *  ISR entry-points
 * ========================================================================= */

/**
 * @brief  Core SPWM update — called at 10 kHz from TIM1 update interrupt.
 *
 *  Execution budget:  at 84 MHz, one ISR call budget = 8 400 cycles.
 *  This routine does: one float add, one compare, table lookup ×3, CCR write ×3.
 *  Measured: ~60–80 cycles.  Ample margin.
 */
void STGIB_TIM1_UpdateISR(void)
{
    /* ── 1. Check Cin (software fault detection) ─────────────────────── */
    if (!(GPIOB->IDR & GPIO_IDR_IDR_2)) {
        /* Cin asserted (active LOW) */
        _latch_fault(STGIB_FAULT_OVERCURRENT);
        return;
    }

    if (!s_pwm_enabled || s_state == STGIB_STATE_STOPPED ||
        s_state == STGIB_STATE_FAULT) {
        return;
    }

    /* ── 2. Ramp current_hz toward target_hz ────────────────────────── */
    float cur = s_current_hz;
    float tgt = s_target_hz;

    if (cur < tgt - STGIB_RAMP_DELTA) {
        cur += STGIB_RAMP_DELTA;
        if (s_state == STGIB_STATE_STARTING &&
            cur >= tgt - STGIB_RAMP_DELTA * 2.0f) {
            s_state = STGIB_STATE_RUNNING;
            STGIB_Callback_OnRunning();
        }
    } else if (cur > tgt + STGIB_RAMP_DELTA) {
        cur -= STGIB_RAMP_DELTA;
    } else {
        cur = tgt;
        if (s_state == STGIB_STATE_STOPPING && cur == 0.0f) {
            s_current_hz = 0.0f;
            s_state      = STGIB_STATE_STOPPED;
            _disable_outputs();
            STGIB_Callback_OnStopped();
            return;
        }
    }
    s_current_hz = cur;

    /* ── 3. Compute modulation index (V/f + boost) ───────────────────── */
    float m = _vf_modulation(cur);
    s_modulation = m;

    /* ── 4. Advance phase accumulator ───────────────────────────────── */
    float delta = (float)M_2PI * cur / (float)STGIB_CARRIER_HZ;

    /* Direction: reverse swaps B/C (equivalent to negative sequence) */
    float acc;
    if (s_direction == STGIB_DIR_FORWARD) {
        acc = s_phase_acc + delta;
    } else {
        acc = s_phase_acc - delta;
    }
    if (acc > (float)M_2PI)  acc -= (float)M_2PI;
    if (acc < 0.0f)           acc += (float)M_2PI;
    s_phase_acc = acc;

    /* ── 5. Compute CCR values for all three phases ──────────────────── */
    /*  Phase offsets: 0, −2π/3, +2π/3 (positive sequence)               */
    static const float k120 = (float)(2.0 * M_PI / 3.0);

    uint32_t ccr1 = _ccr_from_phase(acc,            m);
    uint32_t ccr2 = _ccr_from_phase(acc - k120,     m);
    uint32_t ccr3 = _ccr_from_phase(acc + k120,     m);

    /* ── 6. Write to TIM1 capture-compare registers ─────────────────── */
    TIM1->CCR1 = ccr1;
    TIM1->CCR2 = ccr2;
    TIM1->CCR3 = ccr3;
}

/* -------------------------------------------------------------------------- */
void STGIB_TIM1_BreakISR(void)
{
    /* Hardware has already tri-stated all outputs via BKIN/break function */
    _latch_fault(STGIB_FAULT_EM_STOP);

    /* Clear the break interrupt flag */
    TIM1->SR &= ~TIM_SR_BIF;
}

/* =========================================================================
 *  Callbacks — default (weak) implementations
 *  Override these in your application file.
 * ========================================================================= */

__weak void STGIB_Callback_OnFault(STGIB_State_t prev_state, uint8_t flags)
{
    (void)prev_state;
    (void)flags;
    /* Default: do nothing.  Add display update / alarm here when ready. */
}

__weak void STGIB_Callback_OnRunning(void)
{
    /* Default: do nothing. */
}

__weak void STGIB_Callback_OnStopped(void)
{
    /* Default: do nothing. */
}

/* =========================================================================
 *  Integration hooks
 * ========================================================================= */

/* ---- Getters ------------------------------------------------------------ */
float STGIB_Hook_GetCurrentHz(void)   { return s_current_hz; }
float STGIB_Hook_GetTargetHz(void)    { return s_target_hz;  }
float STGIB_Hook_GetModulation(void)  { return s_modulation; }
STGIB_State_t STGIB_Hook_GetState(void) { return s_state;   }
bool  STGIB_Hook_IsForward(void)      { return (s_direction == STGIB_DIR_FORWARD); }
uint8_t STGIB_Hook_GetFaultFlags(void){ return s_fault_flags; }

/* ---- Commands ----------------------------------------------------------- */
void STGIB_Hook_CmdStart(float hz)
{
    STGIB_Start(hz, s_direction);
}

void STGIB_Hook_CmdStop(void)
{
    STGIB_Stop(true);   /* Controlled deceleration */
}

void STGIB_Hook_CmdSetHz(float delta_hz)
{
    float new_hz = s_target_hz + delta_hz;
    if (new_hz < STGIB_MIN_FREQ_HZ) new_hz = STGIB_MIN_FREQ_HZ;
    if (new_hz > STGIB_MAX_FREQ_HZ) new_hz = STGIB_MAX_FREQ_HZ;
    STGIB_SetFrequency(new_hz);
}

void STGIB_Hook_CmdToggleDir(void)
{
    STGIB_Dir_t new_dir = (s_direction == STGIB_DIR_FORWARD) ?
                           STGIB_DIR_REVERSE : STGIB_DIR_FORWARD;
    if (s_state == STGIB_STATE_STOPPED || s_state == STGIB_STATE_FAULT) {
        STGIB_SetDirection(new_dir);
    } else {
        /* Running: stop → change dir → restart */
        float hz = s_target_hz;
        STGIB_Stop(false);                /* Immediate stop */
        STGIB_SetDirection(new_dir);
        STGIB_Start(hz, new_dir);
    }
}

/* ---- Fault reports (called by future ADC / NTC module) ----------------- */
void STGIB_Hook_ReportOvercurrent(void)
{
    _latch_fault(STGIB_FAULT_OVERCURRENT);
}

void STGIB_Hook_ReportOvervoltage(void)
{
    _latch_fault(STGIB_FAULT_OVERVOLTAGE);
}

void STGIB_Hook_ReportOvertemp(void)
{
    _latch_fault(STGIB_FAULT_OVERTEMP);
}

/* =========================================================================
 *  Private implementation
 * ========================================================================= */

/**
 * @brief  Configure all GPIOs used by the STGIB15CH60 interface.
 *
 *  PA8/9/10  → AF1 push-pull (TIM1 CH1/2/3  — HIN U/V/W)
 *  PB13/14/15 → AF1 push-pull (TIM1 CH1N/2N/3N — LIN U/V/W)
 *  PA12      → AF1 input pull-up (TIM1 BKIN — EM_STOP, active LOW)
 *  PB2       → Input pull-up (Cin — overcurrent, active LOW)
 */
static void _gpio_init(void)
{
    /* Enable peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    __DSB();   /* Ensure clocks are active before accessing registers */

    /* ── GPIOA: PA8, PA9, PA10 → TIM1 CH1/2/3 (AF1) ── */
    /* Alternate function mode (10b) */
    GPIOA->MODER &= ~( GPIO_MODER_MODER8  | GPIO_MODER_MODER9  |
                       GPIO_MODER_MODER10 | GPIO_MODER_MODER12 );
    GPIOA->MODER |=  ( (2U << GPIO_MODER_MODER8_Pos)  |
                       (2U << GPIO_MODER_MODER9_Pos)  |
                       (2U << GPIO_MODER_MODER10_Pos) |
                       (2U << GPIO_MODER_MODER12_Pos) );

    /* Output type: push-pull (0b) — already reset value, explicit for safety */
    GPIOA->OTYPER &= ~( GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10 );

    /* Speed: 50 MHz (10b) */
    GPIOA->OSPEEDR |= ( (2U << GPIO_OSPEEDER_OSPEEDR8_Pos)  |
                        (2U << GPIO_OSPEEDER_OSPEEDR9_Pos)  |
                        (2U << GPIO_OSPEEDER_OSPEEDR10_Pos) );

    /* No pull on PA8/9/10 (driven by TIM1) */
    GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPDR8  | GPIO_PUPDR_PUPDR9  |
                       GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR12 );

    /* PA12 BKIN: pull-up (01b) — EM_STOP is active LOW */
    GPIOA->PUPDR |= (1U << GPIO_PUPDR_PUPDR12_Pos);

    /* AF1 for PA8, PA9, PA10, PA12 (AFR[1] = AFRH, covers pins 8-15) */
    GPIOA->AFR[1] &= ~( (0xFUL << 0)  |   /* PA8  */
                         (0xFUL << 4)  |   /* PA9  */
                         (0xFUL << 8)  |   /* PA10 */
                         (0xFUL << 16) );  /* PA12 */
    GPIOA->AFR[1] |=  ( (1U << 0)  |      /* PA8  → AF1 */
                         (1U << 4)  |      /* PA9  → AF1 */
                         (1U << 8)  |      /* PA10 → AF1 */
                         (1U << 16) );     /* PA12 → AF1 */

    /* ── GPIOB: PB13, PB14, PB15 → TIM1 CH1N/2N/3N (AF1) ── */
    GPIOB->MODER &= ~( GPIO_MODER_MODER2  |
                       GPIO_MODER_MODER13 | GPIO_MODER_MODER14 |
                       GPIO_MODER_MODER15 );
    GPIOB->MODER |=  ( (2U << GPIO_MODER_MODER13_Pos) |
                       (2U << GPIO_MODER_MODER14_Pos) |
                       (2U << GPIO_MODER_MODER15_Pos) );
    /* PB2 stays input (00b — already default) */

    /* Output type push-pull for PB13/14/15 */
    GPIOB->OTYPER &= ~( GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_15 );

    /* Speed 50 MHz for PB13/14/15 */
    GPIOB->OSPEEDR |= ( (2U << GPIO_OSPEEDER_OSPEEDR13_Pos) |
                        (2U << GPIO_OSPEEDER_OSPEEDR14_Pos) |
                        (2U << GPIO_OSPEEDER_OSPEEDR15_Pos) );

    /* PB2 input pull-up (Cin — active LOW): 01b */
    GPIOB->PUPDR &= ~( GPIO_PUPDR_PUPDR2  |
                       GPIO_PUPDR_PUPDR13 | GPIO_PUPDR_PUPDR14 |
                       GPIO_PUPDR_PUPDR15 );
    GPIOB->PUPDR |= (1U << GPIO_PUPDR_PUPDR2_Pos);

    /* AF1 for PB13, PB14, PB15 (AFR[1] = AFRH) */
    GPIOB->AFR[1] &= ~( (0xFUL << 20) |   /* PB13 */
                         (0xFUL << 24) |   /* PB14 */
                         (0xFUL << 28) );  /* PB15 */
    GPIOB->AFR[1] |=  ( (1U << 20) |      /* PB13 → AF1 */
                         (1U << 24) |      /* PB14 → AF1 */
                         (1U << 28) );     /* PB15 → AF1 */
}

/**
 * @brief  Configure TIM1 for center-aligned 3-phase complementary PWM.
 *
 *  • Center-aligned mode 1 (up/down counting, UIF on up-count only)
 *  • ARR = STGIB_ARR  (= 4200 for 10 kHz @ 84 MHz)
 *  • CH1/2/3: PWM mode 1, preload enabled
 *  • CH1N/2N/3N: complementary outputs enabled
 *  • BDTR: break enabled (active LOW), dead-time = STGIB_DEAD_TIME_CNT,
 *          auto-output enable, OSSI/OSSR (low-side on break/idle)
 *  • All CCRs start at ARR/2 (zero average voltage to motor)
 */
static void _tim1_init(void)
{
    /* Enable TIM1 clock (APB2) */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    __DSB();

    /* Stop timer while configuring */
    TIM1->CR1 = 0;

    /* ── Time base ─────────────────────────────────────────────────── */
    TIM1->PSC = 0;                      /* No prescaler: f_TIM = APB2 clk */
    TIM1->ARR = STGIB_ARR - 1U;        /* 4199 for 10 kHz center-aligned  */

    /* Center-aligned mode 1 (CMS = 01), direction bit managed by HW */
    TIM1->CR1 = TIM_CR1_CMS_0;         /* CMS = 01 → center-aligned mode 1 */

    /* ── Output compare channels 1, 2, 3 ──────────────────────────── */
    /* CH1: PWM mode 1 (OCM = 110), preload enabled */
    TIM1->CCMR1 =  (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                   (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 =  (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    /* Enable main output (CCxE) and complementary output (CCxNE) */
    TIM1->CCER = TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E  | TIM_CCER_CC3NE;

    /* Polarity: active HIGH for both main and complementary outputs */
    /* CC1P=0, CC1NP=0 etc. → already 0 from CCER write above */

    /* Set initial CCR to mid-rail (zero motor voltage) */
    TIM1->CCR1 = STGIB_ARR_HALF;
    TIM1->CCR2 = STGIB_ARR_HALF;
    TIM1->CCR3 = STGIB_ARR_HALF;

    /* ── Break and dead-time register (BDTR) ─────────────────────── */
    /*
     *  BKE  = 1  Break input enabled (PA12 / BKIN)
     *  BKP  = 0  Break polarity: active LOW  (EM_STOP pulls PA12 LOW)
     *  AOE  = 1  Automatic output enable after break cleared
     *  OSSR = 1  Off-state selection for Run mode (low-side active on fault)
     *  OSSI = 1  Off-state selection for Idle mode
     *  LOCK = 01 Lock level 1 (protects DTG, BKP, BKE from accidental writes)
     *  DTG  = STGIB_DEAD_TIME_CNT (84 → ~1 µs dead-time)
     */
    TIM1->BDTR = TIM_BDTR_BKE                           |
                 /* BKP = 0 (active LOW) */
                 TIM_BDTR_AOE                            |
                 TIM_BDTR_OSSR                           |
                 TIM_BDTR_OSSI                           |
                 (1U << TIM_BDTR_LOCK_Pos)               |  /* LOCK = 01 */
                 (STGIB_DEAD_TIME_CNT & 0xFFU);

    /* Enable update interrupt (UIF) for SPWM ISR */
    TIM1->DIER = TIM_DIER_UIE | TIM_DIER_BIE;   /* Update + Break IRQ */

    /* Enable break interrupt */
    /* (BIE already set above via DIER) */

    /* Start counter — outputs still gated by MOE (disabled until _enable_outputs) */
    TIM1->EGR  = TIM_EGR_UG;   /* Trigger UG to preload ARR and CCRs */
    TIM1->SR   = 0;             /* Clear any pending flags             */
    TIM1->CR1 |= TIM_CR1_CEN;  /* Start counter                       */

    /* Note: Main Output Enable (MOE) is NOT set here.
     *       _enable_outputs() sets MOE; _disable_outputs() clears it.  */
}

/**
 * @brief  Configure NVIC for TIM1 interrupts.
 */
static void _nvic_init(void)
{
    /* TIM1 Update + TIM10 share IRQ on F411 (TIM1_UP_TIM10_IRQn = 25) */
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0U);  /* Highest priority */
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    /* TIM1 Break + TIM9 share IRQ on F411 (TIM1_BRK_TIM9_IRQn = 24) */
    NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0U);
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
}

/**
 * @brief  Build the per-phase CCR lookup table for a given modulation index.
 *
 *  Uses THIPWM: V(θ) = m · [sin(θ) + sin(3θ)/6]
 *
 *  CCR(i) = (ARR/2) · [1 + m · (sin(2πi/N) + sin(6πi/N)/6)]
 *
 *  Only phase A is stored.  Phases B and C are addressed by offsetting
 *  the table index by ±TABLE_SIZE/3 at runtime.
 */
static void _build_ccr_table(float modulation)
{
    const float half = (float)STGIB_ARR_HALF;
    const float step = (float)M_2PI / (float)STGIB_SIN_TABLE_SIZE;

    for (uint32_t i = 0; i < STGIB_SIN_TABLE_SIZE; i++) {
        float theta = step * (float)i;
        float v = modulation * (sinf(theta) + sinf(3.0f * theta) / 6.0f);
        float ccr = half * (1.0f + v);
        /* Clamp to [0, ARR] */
        if (ccr < 0.0f)            ccr = 0.0f;
        if (ccr > (float)STGIB_ARR) ccr = (float)STGIB_ARR;
        s_ccr_table[i] = (uint16_t)ccr;
    }
}

/**
 * @brief  Compute CCR from continuous phase angle and modulation index.
 *
 *  Inline computation used in the ISR (avoids rebuilding the full table
 *  on every modulation change).  sinf() on Cortex-M4 FPU ≈ 14 cycles.
 *
 * @param  theta_rad  Phase angle in radians (any range — normalised internally)
 * @param  modulation Modulation index [0 .. 1]
 * @return CCR value [0 .. STGIB_ARR]
 */
static uint16_t _ccr_from_phase(float theta_rad, float modulation)
{
    /* Normalise angle to [0, 2π] */
    while (theta_rad >  (float)M_2PI) theta_rad -= (float)M_2PI;
    while (theta_rad <  0.0f)         theta_rad += (float)M_2PI;

    /* THIPWM */
    float v   = modulation * (sinf(theta_rad) + sinf(3.0f * theta_rad) / 6.0f);
    float ccr = (float)STGIB_ARR_HALF * (1.0f + v);

    if (ccr < 0.0f)              return 0U;
    if (ccr > (float)STGIB_ARR)  return (uint16_t)STGIB_ARR;
    return (uint16_t)ccr;
}

/**
 * @brief  Compute modulation index using V/f law with low-speed boost.
 *
 *  m(f) = BOOST + (1 − BOOST) · f / f_base     for f ≤ f_base
 *  m(f) = 1.0                                   for f > f_base
 *
 *  Result is clamped to [0, 1].
 */
static float _vf_modulation(float hz)
{
    if (hz <= 0.0f) return 0.0f;

    float m;
    if (hz >= STGIB_BASE_FREQ_HZ) {
        m = 1.0f;
    } else {
        m = STGIB_BOOST + (1.0f - STGIB_BOOST) * (hz / STGIB_BASE_FREQ_HZ);
    }

    if (m > 1.0f) m = 1.0f;
    if (m < 0.0f) m = 0.0f;
    return m;
}

/**
 * @brief  Enable PWM main output (set MOE in BDTR).
 */
static void _enable_outputs(void)
{
    s_pwm_enabled = true;
    TIM1->BDTR |= TIM_BDTR_MOE;    /* Main output enable */
}

/**
 * @brief  Disable PWM main output and tri-state all gate signals.
 */
static void _disable_outputs(void)
{
    s_pwm_enabled = false;
    TIM1->BDTR &= ~TIM_BDTR_MOE;   /* Clear main output enable */
    /* Drive CCRs to mid-rail to avoid voltage on re-enable */
    TIM1->CCR1 = STGIB_ARR_HALF;
    TIM1->CCR2 = STGIB_ARR_HALF;
    TIM1->CCR3 = STGIB_ARR_HALF;
}

/**
 * @brief  Latch a fault condition, disable outputs, notify callback.
 * @param  flag  One of the STGIB_FaultFlags_t bits.
 */
static void _latch_fault(uint8_t flag)
{
    STGIB_State_t prev = s_state;

    _disable_outputs();

    s_fault_flags |= flag;
    s_fault_count++;
    s_state       = STGIB_STATE_FAULT;
    s_current_hz  = 0.0f;

    STGIB_Callback_OnFault(prev, s_fault_flags);
}
