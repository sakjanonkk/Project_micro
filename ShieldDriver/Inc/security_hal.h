// security_hal.h

#ifndef SECURITY_HAL_H_
#define SECURITY_HAL_H_

#include <stdint.h>
#include <stdbool.h>

// --- Initialization ---
void HAL_Init(void);

// --- Input Functions ---
float   HAL_GetTemperature(void);      // Returns temperature in Celsius
uint16_t HAL_GetPotValue(void);        // Returns raw ADC value (0-4095)
bool    HAL_IsIntrusionDetected(void);
bool    HAL_IsS1Pressed(void);
bool    HAL_IsS2Pressed(void);
bool    HAL_IsS3Pressed(void);
bool HAL_IsYellowLED_On(void);

// --- Output Functions ---
void HAL_SetRedLED(bool state);
void HAL_SetYellowLED(bool state);
void HAL_SetGB_LED(bool state);
void HAL_SetBuzzer(bool state);

// --- Combined Alarm Functions ---
void HAL_ActivateFireAlarm(void);
void HAL_ActivateIntruderAlarm(void);
void HAL_DeactivateAllAlarms(void);

// --- Utility ---
void HAL_Delay(uint32_t ms);


#endif /* SECURITY_HAL_H_ */
