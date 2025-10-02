// security_hal.h

#ifndef SECURITY_HAL_H_
#define SECURITY_HAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f411xe.h"

void Security_HAL_Init(void);

// Input Functions
float    HAL_GetTemperature(void);
uint16_t HAL_GetPotValue(void);
bool     HAL_IsIntrusionDetected(void);
bool     HAL_IsS1Pressed(void);
bool     HAL_IsS2Pressed(void);
bool     HAL_IsS3Pressed(void);
bool     HAL_IsModeSelectLED_On(void);
bool     HAL_IsFireAlarmLED_On(void);

// Output Functions
void HAL_SetFireAlarmLED(bool state);
void HAL_SetModeSelectLED(bool state);
void HAL_SetHomeModeLED(bool state);
void HAL_SetAwayModeLED(bool state);
void HAL_SetBuzzer(bool state);

// Combined Alarm Functions
void HAL_ActivateFireAlarm(void);
void HAL_ActivateIntruderAlarm(void);
void HAL_DeactivateAllAlarms(void);

// Utility
uint32_t HAL_GetTick(void);
void SysTick_Handler(void);
void HAL_UART_TxString(char strOut[]);

#endif /* SECURITY_HAL_H_ */
