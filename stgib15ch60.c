/**
 ******************************************************************************
 * @file    stgib15ch60_irq.c
 * @brief   TIM1 IRQ glue for STGIB15CH60 driver (safe to add without editing
 *          CubeMX-generated stm32f4xx_it.c).
 *
 * Notes:
 *  - Handlers are declared with __attribute__((weak)). If your project or
 *    CubeMX already provides TIM1_UP_TIM10_IRQHandler / TIM1_BRK_TIM9_IRQHandler
 *    (strong symbols), those will take precedence and no link conflict occurs.
 *  - If CubeMX provides its own handlers, you may (optionally) call the
 *    helper functions STGIB_TIM1_UpdateIRQHandler() and STGIB_TIM1_BreakIRQHandler()
 *    from the generated handlers to forward events to the STGIB driver.
 *
 *  - These handlers perform minimal, deterministic work:
 *      • check TIM1 SR flags (UIF / BIF)
 *      • clear flags where appropriate
 *      • call STGIB_TIM1_UpdateISR() or STGIB_TIM1_BreakISR()
 *
 *  - Do NOT call HAL_TIM_IRQHandler from here; the STGIB driver manipulates
 *    TIM1 registers directly for lowest-latency SPWM updates.
 *
 ******************************************************************************
 */

#include "stgib15ch60.h"
#include "stm32f4xx.h"

/* ------------------------------------------------------------------ */
/*  Helper wrapper functions (public)                                 */
/*  These can be called from CubeMX-generated handlers if you prefer  */
/*  not to rely on the weak IRQ definitions.                           */
/* ------------------------------------------------------------------ */

/**
 * @brief  Forward TIM1 update event to the STGIB update ISR.
 *
 * This function performs the same logic the IRQ handler would:
 * - checks UIF in TIM1->SR
 * - clears UIF
 * - calls STGIB_TIM1_UpdateISR()
 *
 * Safe to call from generated TIM1_UP handler.
 */
void STGIB_TIM1_UpdateIRQHandler(void)
{
    /* Check UIF (update interrupt) */
    if (TIM1->SR & TIM_SR_UIF) {
        /* Clear UIF */
        TIM1->SR &= ~TIM_SR_UIF;
        /* Call library ISR */
        STGIB_TIM1_UpdateISR();
    }
}

/**
 * @brief  Forward TIM1 break event to the STGIB break ISR.
 *
 * This function simply calls the library break ISR which latches the fault.
 * The library's STGIB_TIM1_BreakISR() clears BIF internally.
 *
 * Safe to call from generated TIM1_BRK handler.
 */
void STGIB_TIM1_BreakIRQHandler(void)
{
    /* If BIF set, call break ISR. The library will clear the flag as needed. */
    if (TIM1->SR & TIM_SR_BIF) {
        STGIB_TIM1_BreakISR();
    }
}

/* ------------------------------------------------------------------ */
/*  IRQ handlers (weak) — will be used only if no strong handlers exist */
/* ------------------------------------------------------------------ */

/**
 * @brief  TIM1 update interrupt + TIM10 global interrupt handler.
 *
 * Declared weak to avoid multiple-definition when CubeMX provides its own handler.
 * When used, checks UIF and forwards to STGIB_TIM1_UpdateISR().
 */
void TIM1_UP_TIM10_IRQHandler(void) __attribute__((weak));
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* Forward to helper which checks UIF and calls the library ISR */
    STGIB_TIM1_UpdateIRQHandler();

    /* Note:
     *  - We do NOT call HAL_TIM_IRQHandler here because the STGIB driver
     *    uses direct register access to achieve predictable low-latency updates.
     *  - If your project relies on HAL timer handling for other timers,
     *    keep those in their own handlers (this file only touches TIM1 SR).
     */
}

/**
 * @brief  TIM1 Break interrupt + TIM9 global interrupt handler.
 *
 * Declared weak to avoid multiple-definition when CubeMX provides its own handler.
 * When used, checks BIF and forwards to STGIB_TIM1_BreakISR().
 */
void TIM1_BRK_TIM9_IRQHandler(void) __attribute__((weak));
void TIM1_BRK_TIM9_IRQHandler(void)
{
    STGIB_TIM1_BreakIRQHandler();
}