/**
 ******************************************************************************
 * @file    ui_protocol.h
 * @brief   Bluetooth UI Protocol — Command & Status Telemetry
 *
 * Protocol format: "KEY:VALUE\n"
 *
 * Tx (Status telemetry sent to app):
 *   sp:XXXX    — Motor speed (RPM) - not directly available, use Hz
 *   tr:X.XX    — Torque reference (0.0 ~ 1.0 normalized)
 *   fr:XX.X    — Output frequency (Hz)
 *   di:X       — Direction (0=stopped, 1=forward, 2=reverse)
 *   po:X       — Power factor (0~10, ×0.1)
 *   NT:XX.X    — Temperature of IPM/motor (°C)
 *   vb:XXX.X   — DC-bus voltage (V)
 *   ib:X.X     — DC-bus current (A)
 *   iu:X.XX    — Phase U current (A)
 *   iv:X.XX    — Phase V current (A)
 *   iw:X.XX    — Phase W current (A)
 *   vu:XXX     — Phase U voltage (RMS)
 *   vv:XXX     — Phase V voltage (RMS)
 *   vw:XXX     — Phase W voltage (RMS)
 *   fl:X       — Fault flags (bitmask: 0=none, 1=OC, 2=EM_STOP, 4=Temp, 8=OV)
 *   br:X       — Break reason on fault (0=no, 1=fault, 2=user_stop)
 *
 * Rx (Commands received from app):
 *   CMD:ST:F   — Start motor, Forward direction
 *   CMD:ST:R   — Start motor, Reverse direction
 *   CMD:SP:XX.X — Set speed (Hz)
 *   CMD:STP    — Stop motor (soft ramp)
 *   CMD:EMG    — Emergency stop
 *   CMD:CLR    — Clear fault
 *   CMD:DIR:F  — Set direction Forward (only when stopped)
 *   CMD:DIR:R  — Set direction Reverse (only when stopped)
 *
 * ────────────────────────────────────────────────────────────────────────
 ******************************************************************************
 */

#ifndef UI_PROTOCOL_H
#define UI_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 *  Types
 * =========================================================================== */

/** Sensor readings — captured atomically from ADC and driver state */
typedef struct {
    /* Motor control */
    float           frequency_hz;      /**< Current output frequency (Hz)   */
    float           target_hz;         /**< Target frequency (Hz)           */
    float           modulation;        /**< Modulation index (0..1)         */
    uint8_t         direction;         /**< 0=stopped, 1=forward, 2=reverse */
    uint8_t         state;             /**< Motor state (stopped/running)   */
    
    /* Electrical measurements */
    float           bus_voltage;       /**< DC-link voltage (V)             */
    float           bus_current;       /**< DC-link current (A)             */
    float           phase_u_current;   /**< Phase U current (A)             */
    float           phase_v_current;   /**< Phase V current (A)             */
    float           phase_w_current;   /**< Phase W current (A)             */
    float           phase_u_voltage;   /**< Phase U voltage RMS (V)         */
    float           phase_v_voltage;   /**< Phase V voltage RMS (V)         */
    float           phase_w_voltage;   /**< Phase W voltage RMS (V)         */
    
    /* Thermal & fault */
    float           temperature;       /**< IPM/motor temperature (°C)      */
    uint8_t         fault_flags;       /**< Bitmask of active faults        */
    uint8_t         break_reason;      /**< Why motor stopped (0=none, 1=fault, 2=user) */
    float           power_factor;      /**< Multiplied by 10 for 0.1 precision */
} UI_SensorSnapshot_t;

/** Command from Bluetooth app */
typedef enum {
    UI_CMD_NONE = 0,
    UI_CMD_START_FWD,       /**< Start forward     */
    UI_CMD_START_REV,       /**< Start reverse     */
    UI_CMD_SET_FREQUENCY,   /**< Set Hz target     */
    UI_CMD_STOP,            /**< Soft stop (ramp)  */
    UI_CMD_EMERGENCY_STOP,  /**< Hard stop (coast) */
    UI_CMD_CLEAR_FAULT,     /**< Clear latched fault */
    UI_CMD_SET_DIR_FWD,     /**< Set direction Fwd (when stopped) */
    UI_CMD_SET_DIR_REV,     /**< Set direction Rev (when stopped) */
} UI_CommandType_t;

typedef struct {
    UI_CommandType_t type;
    float            param_hz;  /**< For SET_FREQUENCY command */
} UI_Command_t;

/* ===========================================================================
 *  Public API — Tx (Status/Telemetry)
 * =========================================================================== */

/**
 * @brief  Capture current sensor/driver state atomically.
 * @return Snapshot of all monitored values (safe to read in main loop)
 *
 * Call this periodically (e.g., 100 ms) to avoid stale data.
 */
UI_SensorSnapshot_t UI_GetSnapshot(void);

/**
 * @brief  Format snapshot as Bluetooth tx string.
 * @param  snap       Sensor snapshot from UI_GetSnapshot()
 * @param  tx_buffer  Preallocated string buffer (at least 512 bytes)
 * @param  max_len    Size of tx_buffer
 * @return Number of bytes written (0 if buffer too small)
 *
 * Example output:
 *   sp:1200\n
 *   tr:0.50\n
 *   fr:50.0\n
 *   ... (up to 16 lines)
 */
uint16_t UI_SnapshotToTxString(const UI_SensorSnapshot_t *snap,
                               char *tx_buffer,
                               uint16_t max_len);

/* ===========================================================================
 *  Public API — Rx (Commands)
 * =========================================================================== */

/**
 * @brief  Parse one line of received Bluetooth data.
 *
 * @param  rx_line   Null-terminated string, e.g. "CMD:ST:F\n" or "CMD:SP:35.5"
 * @return Parsed command (type=UI_CMD_NONE if unrecognized)
 *
 * Recognized formats:
 *   "CMD:ST:F"     → UI_CMD_START_FWD
 *   "CMD:ST:R"     → UI_CMD_START_REV
 *   "CMD:SP:XX.X"  → UI_CMD_SET_FREQUENCY (param_hz = XX.X)
 *   "CMD:STP"      → UI_CMD_STOP
 *   "CMD:EMG"      → UI_CMD_EMERGENCY_STOP
 *   "CMD:CLR"      → UI_CMD_CLEAR_FAULT
 *   "CMD:DIR:F"    → UI_CMD_SET_DIR_FWD
 *   "CMD:DIR:R"    → UI_CMD_SET_DIR_REV
 */
UI_Command_t UI_ParseRxLine(const char *rx_line);

/**
 * @brief  Execute a parsed command on the motor driver.
 * @param  cmd  Command from UI_ParseRxLine()
 * @return true if command executed, false if invalid/ignored
 */
bool UI_ExecuteCommand(const UI_Command_t *cmd);

/* ===========================================================================
 *  ADC/Sensor integration (call from ADC ISR or main loop)
 * =========================================================================== */

/**
 * @brief  Report DC-bus voltage (volts).
 * @param  voltage_v  ADC-measured bus voltage
 */
void UI_SetBusVoltage(float voltage_v);

/**
 * @brief  Report DC-bus current (amps).
 * @param  current_a  ADC-measured bus current
 */
void UI_SetBusCurrent(float current_a);

/**
 * @brief  Report three-phase currents (amps).
 * @param  iu, iv, iw  Phase U, V, W RMS currents
 */
void UI_SetPhaseCurrents(float iu, float iv, float iw);

/**
 * @brief  Report three-phase voltages (RMS volts).
 * @param  vu, vv, vw  Phase U, V, W voltages
 */
void UI_SetPhaseVoltages(float vu, float vv, float vw);

/**
 * @brief  Report IPM/motor temperature (°C).
 * @param  temp_c  NTC sensor temperature
 */
void UI_SetTemperature(float temp_c);

/**
 * @brief  Report break/stop reason.
 * @param  reason  0=none, 1=fault_triggered, 2=user_stop
 */
void UI_SetBreakReason(uint8_t reason);

/**
 * @brief  Compute and set power factor (called after setting voltages/currents).
 *         Formula: PF = (vu*iu + vv*iv + vw*iw) / sqrt(3) / V_rms / I_rms
 */
void UI_ComputePowerFactor(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_PROTOCOL_H */
