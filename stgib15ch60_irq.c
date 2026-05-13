/**
 ******************************************************************************
 * @file    stgib15ch60_irq.c
 * @brief   Interrupt handlers for STGIB15CH60 driver — STM32F411
 *
 *  Include this file in your project (or paste its content into your
 *  existing stm32f4xx_it.c if you already have one).
 *
 *  These two handlers are the ONLY required glue between the STM32 HAL/CMSIS
 *  interrupt vector table and the STGIB15CH60 library.
 ******************************************************************************
 */

#include "stgib15ch60.h"

/**
 * @brief  TIM1 Update interrupt + TIM10 global interrupt handler.
 *
 *  Called at STGIB_CARRIER_HZ (10 000 Hz) by TIM1 update event.
 *  Routes to STGIB_TIM1_UpdateISR() which updates CCR1/2/3 for SPWM.
 *
 *  IRQ number: TIM1_UP_TIM10_IRQn (25) on STM32F411.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR = ~TIM_SR_UIF;   /* Clear update interrupt flag */
        STGIB_TIM1_UpdateISR();
    }
    /* TIM10 UIF (if used) handled here too — not used in this project */
}

/**
 * @brief  TIM1 Break interrupt + TIM9 global interrupt handler.
 *
 *  Called immediately when EM_STOP asserts PA12 LOW.
 *  Hardware already tri-states all gate signals; ISR latches fault in software.
 *
 *  IRQ number: TIM1_BRK_TIM9_IRQn (24) on STM32F411.
 */
void TIM1_BRK_TIM9_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_BIF) {
        STGIB_TIM1_BreakISR();   /* Clears BIF inside */
    }
}


/* ============================================================================
 *  Usage example — paste into your main.c
 * ============================================================================
 *
 * #include "stgib15ch60.h"
 *
 * // Override fault callback (optional)
 * void STGIB_Callback_OnFault(STGIB_State_t prev, uint8_t flags)
 * {
 *     // e.g. light an error LED, log to UART
 * }
 *
 * int main(void)
 * {
 *     // 1. System clock init (configure PLL → 84 MHz)
 *     SystemInit();
 *
 *     // 2. Initialise the IPM interface
 *     STGIB_Init();
 *
 *     // 3. Start the motor at 30 Hz, forward direction
 *     STGIB_Start(30.0f, STGIB_DIR_FORWARD);
 *
 *     while (1)
 *     {
 *         // Check for faults
 *         if (STGIB_IsFault())
 *         {
 *             // ... wait for operator acknowledgement, then:
 *             STGIB_ClearFault();
 *         }
 *
 *         // Read status for display (future: call inside display module)
 *         STGIB_Status_t st = STGIB_GetStatus();
 *         // st.current_hz, st.modulation, st.state, st.fault_flags …
 *
 *         // --- Future connections (add one line each) ---
 *         // Buttons  : call STGIB_Hook_CmdStart/Stop/SetHz/ToggleDir
 *         // Display  : call STGIB_Hook_GetCurrentHz / GetState / IsForward
 *         // ADC/NTC  : call STGIB_Hook_ReportOvercurrent / Overtemp
 *     }
 * }
 * ============================================================================ */
