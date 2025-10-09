// security_hal.c

#include "security_hal.h"
#include <math.h>
#include <stdio.h>

char debug_buffer[100];

// --- Pin Mapping ---
#define FIRE_ALARM_LED_PORT     GPIOA
#define FIRE_ALARM_LED_PIN      6

#define MODE_SELECT_LED_PORT    GPIOA
#define MODE_SELECT_LED_PIN     7

#define HOME_MODE_LED_PORT      GPIOB
#define HOME_MODE_LED_PIN       6

#define AWAY_MODE_LED_PORT      GPIOA
#define AWAY_MODE_LED_PIN       5

#define BUZZER_PORT             GPIOC
#define BUZZER_PIN              11

#define S1_PORT                 GPIOA
#define S1_PIN                  10

#define S2_PORT                 GPIOB
#define S2_PIN                  3

#define S3_PORT                 GPIOB
#define S3_PIN                  5

#define INTRUSION_PORT          GPIOB
#define INTRUSION_PIN           4

#define TEMP_SENSOR_PORT        GPIOA
#define TEMP_SENSOR_PIN         0

#define LDR_SENSOR_PORT         GPIOA      // ⭐ เพิ่ม
#define LDR_SENSOR_PIN          1          // ⭐ เพิ่ม

#define POT_PORT                GPIOA
#define POT_PIN                 4

// --- NTC Constants ---
#define NTC_BETA            3950.0f
#define NTC_R0              10000.0f
#define NTC_T0              298.15f
#define NTC_PULLUP_R        10000.0f
#define ADC_VREF            3.3f
#define ADC_MAX_VAL         4095.0f

// ⭐ เพิ่ม LDR Constants
#define LDR_SLOPE           -0.6875f
#define LDR_OFFSET          5.1276f
#define LDR_PULLUP_R        10000.0f
#define LDR_THRESHOLD       300.0f

volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void) {
    ms_ticks++;
}

uint32_t HAL_GetTick(void) {
    return ms_ticks;
}

static uint16_t read_adc_channel(uint8_t channel) {
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void Security_HAL_Init(void) {
    // Enable Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Clear and Configure Output Pins properly
    FIRE_ALARM_LED_PORT->MODER &= ~(0b11 << (FIRE_ALARM_LED_PIN * 2));
    FIRE_ALARM_LED_PORT->MODER |= (0b01 << (FIRE_ALARM_LED_PIN * 2));

    MODE_SELECT_LED_PORT->MODER &= ~(0b11 << (MODE_SELECT_LED_PIN * 2));
    MODE_SELECT_LED_PORT->MODER |= (0b01 << (MODE_SELECT_LED_PIN * 2));

    HOME_MODE_LED_PORT->MODER &= ~(0b11 << (HOME_MODE_LED_PIN * 2));
    HOME_MODE_LED_PORT->MODER |= (0b01 << (HOME_MODE_LED_PIN * 2));

    AWAY_MODE_LED_PORT->MODER &= ~(0b11 << (AWAY_MODE_LED_PIN * 2));
    AWAY_MODE_LED_PORT->MODER |= (0b01 << (AWAY_MODE_LED_PIN * 2));

    // Configure Buzzer pin (ไม่แก้ - ทำงานแล้ว!)
    BUZZER_PORT->MODER &= ~(0b11 << (BUZZER_PIN * 2));
    BUZZER_PORT->MODER |= (0b01 << (BUZZER_PIN * 2));
    BUZZER_PORT->OTYPER &= ~(1 << BUZZER_PIN);
    BUZZER_PORT->OSPEEDR &= ~(0b11 << (BUZZER_PIN * 2));
    BUZZER_PORT->PUPDR &= ~(0b11 << (BUZZER_PIN * 2));

    // Initialize all outputs to OFF
    FIRE_ALARM_LED_PORT->BSRR = (1 << (FIRE_ALARM_LED_PIN + 16));
    MODE_SELECT_LED_PORT->BSRR = (1 << (MODE_SELECT_LED_PIN + 16));
    HOME_MODE_LED_PORT->BSRR = (1 << (HOME_MODE_LED_PIN + 16));
    AWAY_MODE_LED_PORT->BSRR = (1 << (AWAY_MODE_LED_PIN + 16));
    BUZZER_PORT->BSRR = (1 << (BUZZER_PIN + 16));  // ไม่แก้ - ทำงานแล้ว!

    // Configure Input Pins with Pull-up
    S1_PORT->PUPDR |= (0b01 << (S1_PIN * 2));
    S2_PORT->PUPDR |= (0b01 << (S2_PIN * 2));
    S3_PORT->PUPDR |= (0b01 << (S3_PIN * 2));
    INTRUSION_PORT->PUPDR |= (0b01 << (INTRUSION_PIN * 2));

    // Configure Analog Pins
    TEMP_SENSOR_PORT->MODER |= (0b11 << (TEMP_SENSOR_PIN * 2));
    LDR_SENSOR_PORT->MODER |= (0b11 << (LDR_SENSOR_PIN * 2));  // ⭐ เพิ่ม LDR
    POT_PORT->MODER |= (0b11 << (POT_PIN * 2));

    // Configure ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // Configure SysTick Timer for 1ms interrupts
    SysTick_Config(16000000 / 1000);

    // Configure UART2 for Debugging (PA2=TX, PA3=RX)
    GPIOA->MODER |= (0b10 << (2*2)) | (0b10 << (3*2));
    GPIOA->AFR[0] |= (7 << (2*4)) | (7 << (3*4));
    USART2->BRR = 16000000 / 115200;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;

    // Enable FPU for floating point math
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

float HAL_GetTemperature(void) {
    uint16_t adc_val = read_adc_channel(TEMP_SENSOR_PIN);
    if (adc_val == 0 || adc_val >= 4095) return (adc_val == 0) ? -273.15f : 999.0f;
    float v_out = (adc_val * ADC_VREF) / ADC_MAX_VAL;
    float r_ntc = (v_out * NTC_PULLUP_R) / (ADC_VREF - v_out);
    float temp_k = (NTC_BETA * NTC_T0) / (NTC_T0 * logf(r_ntc / NTC_R0) + NTC_BETA);
    return temp_k - 273.15f;
}

// ⭐ เพิ่มฟังก์ชัน LDR
uint16_t HAL_GetLightLevel(void) {
    uint16_t adc_val = read_adc_channel(LDR_SENSOR_PIN);
    if (adc_val == 0 || adc_val >= 4095) return 0;

    float v_out = (adc_val * ADC_VREF) / ADC_MAX_VAL;
    float r_ldr = (v_out * LDR_PULLUP_R) / (ADC_VREF - v_out);

    if (r_ldr <= 0) return 0;

    float lux = powf(10.0f, (log10f(r_ldr) - LDR_OFFSET) / LDR_SLOPE);
    return (uint16_t)lux;
}

uint16_t HAL_GetPotValue(void) {
    return read_adc_channel(POT_PIN);
}

// ⭐ แก้ให้รวม LDR detection
bool HAL_IsIntrusionDetected(void) {
    bool reed_open = !(INTRUSION_PORT->IDR & (1 << INTRUSION_PIN));
    bool light_changed = (HAL_GetLightLevel() > LDR_THRESHOLD);
    return (reed_open || light_changed);
}

bool HAL_IsS1Pressed(void) { return !(S1_PORT->IDR & (1 << S1_PIN)); }
bool HAL_IsS2Pressed(void) { return !(S2_PORT->IDR & (1 << S2_PIN)); }
bool HAL_IsS3Pressed(void) { return !(S3_PORT->IDR & (1 << S3_PIN)); }

bool HAL_IsModeSelectLED_On(void) { return (MODE_SELECT_LED_PORT->ODR & (1 << MODE_SELECT_LED_PIN)); }
bool HAL_IsFireAlarmLED_On(void) { return (FIRE_ALARM_LED_PORT->ODR & (1 << FIRE_ALARM_LED_PIN)); }

void HAL_SetFireAlarmLED(bool state) { if (state) FIRE_ALARM_LED_PORT->BSRR = (1 << FIRE_ALARM_LED_PIN); else FIRE_ALARM_LED_PORT->BSRR = (1 << (FIRE_ALARM_LED_PIN + 16)); }
void HAL_SetModeSelectLED(bool state) { if (state) MODE_SELECT_LED_PORT->BSRR = (1 << MODE_SELECT_LED_PIN); else MODE_SELECT_LED_PORT->BSRR = (1 << (MODE_SELECT_LED_PIN + 16)); }
void HAL_SetHomeModeLED(bool state) { if (state) HOME_MODE_LED_PORT->BSRR = (1 << HOME_MODE_LED_PIN); else HOME_MODE_LED_PORT->BSRR = (1 << (HOME_MODE_LED_PIN + 16)); }
void HAL_SetAwayModeLED(bool state) { if (state) AWAY_MODE_LED_PORT->BSRR = (1 << AWAY_MODE_LED_PIN); else AWAY_MODE_LED_PORT->BSRR = (1 << (AWAY_MODE_LED_PIN + 16)); }

// ไม่แก้ Buzzer - ทำงานแล้ว!
void HAL_SetBuzzer(bool state) {
    if (state)
        BUZZER_PORT->BSRR = (1 << BUZZER_PIN);
    else
        BUZZER_PORT->BSRR = (1 << (BUZZER_PIN + 16));
}

void HAL_ActivateFireAlarm(void) { HAL_SetFireAlarmLED(true); HAL_SetBuzzer(true); }
void HAL_ActivateIntruderAlarm(void) { HAL_SetBuzzer(true); }

void HAL_DeactivateAllAlarms(void) {
    HAL_SetFireAlarmLED(false);
    HAL_SetModeSelectLED(false);
    HAL_SetHomeModeLED(false);
    HAL_SetAwayModeLED(false);
    HAL_SetBuzzer(false);
}

void HAL_UART_TxString(char strOut[]) {
    for (uint16_t i = 0; strOut[i] != '\0'; i++) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = strOut[i];
    }
}
