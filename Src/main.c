// main.c

#include "security_hal.h"

// System States Enum
typedef enum {
    STATE_DISARMED,
    STATE_MODE_SELECTION,
    STATE_ARMED,
    STATE_INTRUDER_ALERT,
    STATE_FIRE_ALARM
} SystemState_t;

// System Configuration
#define TEMPERATURE_THRESHOLD   55.0f // องศาเซลเซียส
#define PASSCODE_VALUE          2048  // ค่ากลางของ Potentiometer (0-4095)
#define PASSCODE_TOLERANCE      200   // ค่าความคลาดเคลื่อนที่ยอมรับได้

int main(void) {
    // 1. Initialize all hardware
    HAL_Init();

    // 2. Set initial state
    SystemState_t currentState = STATE_DISARMED;
    uint32_t blink_counter = 0;

    // 3. Infinite loop
    while (1) {
        // --- High-Priority Checks (can override any state) ---
        if (HAL_GetTemperature() > TEMPERATURE_THRESHOLD) {
            currentState = STATE_FIRE_ALARM;
        }

        // --- Main State Machine ---
        switch (currentState) {
            case STATE_DISARMED:
                HAL_DeactivateAllAlarms();
                HAL_SetGB_LED(false); // Make sure armed indicator is off

                // Check for user input to start arming sequence
                if (HAL_IsS2Pressed() || HAL_IsS3Pressed()) {
                    currentState = STATE_MODE_SELECTION;
                    HAL_Delay(200); // Debounce delay
                }
                break;

            case STATE_MODE_SELECTION:
                // Blink yellow LED to indicate waiting for passcode
                if (blink_counter % 50 == 0) {
                   HAL_SetYellowLED(!HAL_IsYellowLED_On()); // (You would need to add a HAL_IsYellowLED_On() function for this)
                   // Or simpler:
                   HAL_SetYellowLED(true); HAL_Delay(100); HAL_SetYellowLED(false);
                }

                // Show G/B LED to indicate Home/Away mode (example)
                HAL_SetGB_LED(HAL_IsS2Pressed());

                // Check for arming command
                uint16_t pot_val = HAL_GetPotValue();
                if (HAL_IsS1Pressed() &&
                   (pot_val > (PASSCODE_VALUE - PASSCODE_TOLERANCE)) &&
                   (pot_val < (PASSCODE_VALUE + PASSCODE_TOLERANCE)) )
                {
                    currentState = STATE_ARMED;
                    HAL_DeactivateAllAlarms(); // Turn off yellow blink
                }
                break;

            case STATE_ARMED:
                HAL_SetGB_LED(true); // Solid G/B light to indicate armed status

                // Check for intrusion
                if (HAL_IsIntrusionDetected()) {
                    currentState = STATE_INTRUDER_ALERT;
                }
                break;

            case STATE_INTRUDER_ALERT:
                HAL_ActivateIntruderAlarm();

                // Check for disarm attempt
                uint16_t disarm_pot_val = HAL_GetPotValue();
                if (HAL_IsS1Pressed() &&
                   (disarm_pot_val > (PASSCODE_VALUE - PASSCODE_TOLERANCE)) &&
                   (disarm_pot_val < (PASSCODE_VALUE + PASSCODE_TOLERANCE)) )
                {
                   currentState = STATE_DISARMED;
                   HAL_DeactivateAllAlarms();
                   HAL_Delay(1000); // Delay to prevent re-arming immediately
                }
                break;

            case STATE_FIRE_ALARM:
                HAL_ActivateFireAlarm();

                // Red LED should be blinking. Simple way is to toggle in loop.
                if(blink_counter % 25 == 0) { HAL_SetRedLED(true); } else if (blink_counter % 50 == 0) { HAL_SetRedLED(false); }


                // Wait for user to acknowledge
                if (HAL_IsS1Pressed()) {
                    currentState = STATE_DISARMED;
                    HAL_DeactivateAllAlarms();
                    HAL_Delay(1000); // Prevent false triggers
                }
                break;
        }

        blink_counter++;
        HAL_Delay(10); // Main loop delay
    }
    return 0;
}
