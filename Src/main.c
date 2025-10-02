// main.c

#include "security_hal.h"
#include <stdio.h>

extern char debug_buffer[]; // Use global buffer from security_hal.c

typedef enum {
    STATE_DISARMED,       // <-- C compiler จะกำหนดให้มีค่าเป็น 0
    STATE_MODE_SELECTION, // <-- จะมีค่าเป็น 1
    STATE_ARMED,          // <-- จะมีค่าเป็น 2
    STATE_INTRUDER_ALERT, // <-- จะมีค่าเป็น 3
    STATE_FIRE_ALARM      // <-- จะมีค่าเป็น 4
} SystemState_t;

// --- System Configuration ---
#define TEMPERATURE_THRESHOLD   28.0f // <--- ปรับค่านี้เพื่อทดสอบได้ง่าย
#define PASSCODE_VALUE          2048
#define PASSCODE_TOLERANCE      200

int main(void) {
    Security_HAL_Init();

    SystemState_t currentState = STATE_DISARMED;
    bool isHomeMode = true;

    uint32_t last_button_time = 0;
    uint32_t last_blink_time = 0;
    uint32_t last_debug_print_time = 0;
    const uint32_t DEBOUNCE_DELAY = 200;

    while (1) {
        uint32_t now = HAL_GetTick();

        // Print debug info every 1 second
        if (now - last_debug_print_time >= 1000) {
            last_debug_print_time = now;
            float temp = HAL_GetTemperature();
            uint16_t pot = HAL_GetPotValue();
            sprintf(debug_buffer, "Temp: %.2f C, Pot: %u, State: %d\r\n", temp, pot, currentState);
            HAL_UART_TxString(debug_buffer);
        }

        // High-Priority Fire Check (can override most states)
        if (currentState != STATE_FIRE_ALARM && currentState != STATE_INTRUDER_ALERT) {
            if (HAL_GetTemperature() > TEMPERATURE_THRESHOLD) {
                currentState = STATE_FIRE_ALARM;
            }
        }

        switch (currentState) {
            case STATE_DISARMED:
                HAL_DeactivateAllAlarms();
                if ((HAL_IsS2Pressed() || HAL_IsS3Pressed()) && (now - last_button_time > DEBOUNCE_DELAY)) {
                    last_button_time = now;
                    currentState = STATE_MODE_SELECTION;
                    isHomeMode = true;
                }
                break;

            case STATE_MODE_SELECTION:
                if (now - last_blink_time > 500) {
                    last_blink_time = now;
                    HAL_SetModeSelectLED(!HAL_IsModeSelectLED_On());
                }
                HAL_SetHomeModeLED(isHomeMode);
                HAL_SetAwayModeLED(!isHomeMode);

                if ((HAL_IsS2Pressed() || HAL_IsS3Pressed()) && (now - last_button_time > DEBOUNCE_DELAY)) {
                    last_button_time = now;
                    isHomeMode = !isHomeMode;
                }

                uint16_t pot_val = HAL_GetPotValue();
                if (HAL_IsS1Pressed() && (now - last_button_time > DEBOUNCE_DELAY) &&
                    (pot_val > (PASSCODE_VALUE - PASSCODE_TOLERANCE)) &&
                    (pot_val < (PASSCODE_VALUE + PASSCODE_TOLERANCE))) {
                    last_button_time = now;
                    currentState = STATE_ARMED;
                    HAL_DeactivateAllAlarms();
                }
                break;

            case STATE_ARMED:
                HAL_SetHomeModeLED(isHomeMode);
                HAL_SetAwayModeLED(!isHomeMode);

                if (HAL_IsIntrusionDetected()) {
                    currentState = STATE_INTRUDER_ALERT;
                }
                if ((HAL_IsS2Pressed() || HAL_IsS3Pressed()) && (now - last_button_time > DEBOUNCE_DELAY)) {
                    last_button_time = now;
                    currentState = STATE_MODE_SELECTION;
                }
                break;

            case STATE_INTRUDER_ALERT:
                HAL_ActivateIntruderAlarm();
                if (now - last_blink_time > 150) {
                    last_blink_time = now;
                    bool current_led_state = HAL_IsModeSelectLED_On();
                    HAL_SetFireAlarmLED(!current_led_state);
                    HAL_SetModeSelectLED(!current_led_state);
                    HAL_SetHomeModeLED(!current_led_state);
                    HAL_SetAwayModeLED(!current_led_state);
                }

                uint16_t disarm_pot_val = HAL_GetPotValue();
                if (HAL_IsS1Pressed() && (now - last_button_time > DEBOUNCE_DELAY) &&
                    (disarm_pot_val > (PASSCODE_VALUE - PASSCODE_TOLERANCE)) &&
                    (disarm_pot_val < (PASSCODE_VALUE + PASSCODE_TOLERANCE))) {
                   last_button_time = now;
                   currentState = STATE_DISARMED;
                   HAL_DeactivateAllAlarms();
                }
                break;

            case STATE_FIRE_ALARM:
                HAL_SetBuzzer(true);
                if (now - last_blink_time > 250) {
                    last_blink_time = now;
                    HAL_SetFireAlarmLED(!HAL_IsFireAlarmLED_On());
                }

                if (HAL_IsS1Pressed() && (now - last_button_time > DEBOUNCE_DELAY)) {
                    last_button_time = now;
                    currentState = STATE_DISARMED;
                    HAL_DeactivateAllAlarms();
                }
                break;
        }
    } // End of while(1) loop
    // return 0; // This should be outside the while loop, but main should never exit.
}
