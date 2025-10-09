// main.c

#include "security_hal.h"
#include <stdio.h>

extern char debug_buffer[];

typedef enum {
    STATE_DISARMED,
    STATE_MODE_SELECTION,
    STATE_ARMED,
    STATE_INTRUDER_ALERT,
    STATE_FIRE_ALARM
} SystemState_t;

#define TEMPERATURE_THRESHOLD   30.0f
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

        // ⭐ เพิ่ม LDR ใน debug output
        if (now - last_debug_print_time >= 500) {  // เปลี่ยนเป็น 500ms
            last_debug_print_time = now;
            float temp = HAL_GetTemperature();
            uint16_t pot = HAL_GetPotValue();
            uint16_t lux = HAL_GetLightLevel();

            // ⭐ เพิ่ม Reed raw value
            bool reed_raw = (GPIOB->IDR & (1 << 4)) ? 1 : 0;
            bool intrusion = HAL_IsIntrusionDetected();

            sprintf(debug_buffer, "T:%.1f P:%u Lux:%u Reed:%d Intru:%d St:%d\r\n",
                    temp, pot, lux, reed_raw, intrusion, currentState);
            HAL_UART_TxString(debug_buffer);
        }

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
    }
}
