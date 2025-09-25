// security_hal.c

#include "security_hal.h"
#include "stm32f411xe.h"
#include <math.h>

// --- Pin Definitions (กรุณาแก้ไขให้ตรงกับฮาร์ดแวร์จริง) ---
// LEDs
#define RED_LED_PORT        GPIOA
#define RED_LED_PIN         5
#define YELLOW_LED_PORT     GPIOA
#define YELLOW_LED_PIN      6
#define GB_LED_PORT         GPIOA
#define GB_LED_PIN          7

// Buzzer
#define BUZZER_PORT         GPIOB
#define BUZZER_PIN          6

// Buttons (PC13 คือปุ่มฟ้าบนบอร์ด Nucleo)
#define S1_PORT             GPIOC
#define S1_PIN              13
#define S2_PORT             GPIOB
#define S2_PIN              3
#define S3_PORT             GPIOB
#define S3_PIN              5

// Sensors
#define INTRUSION_PORT      GPIOB
#define INTRUSION_PIN       0
#define TEMP_SENSOR_PORT    GPIOA
#define TEMP_SENSOR_PIN     0    // ADC1_IN0
#define POT_PORT            GPIOA
#define POT_PIN             4    // ADC1_IN4

// --- NTC Thermistor Constants ---
#define NTC_BETA            3950.0f
#define NTC_R0              10000.0f
#define NTC_T0              298.15f  // 25 C in Kelvin
#define NTC_PULLUP_R        10000.0f
#define ADC_VREF            3.3f
#define ADC_MAX_VAL         4095.0f

// --- Private Helper Functions ---
static uint16_t read_adc_channel(uint8_t channel) {
    ADC1->SQR3 = channel;               // Select channel
    ADC1->CR2 |= ADC_CR2_SWSTART;       // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));   // Wait for end of conversion
    return ADC1->DR;                    // Return result
}

// --- Initialization Function ---
void HAL_Init(void) {
    // 1. Enable Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 2. Configure Output Pins (LEDs, Buzzer)
    RED_LED_PORT->MODER |= (1 << (RED_LED_PIN * 2));
    YELLOW_LED_PORT->MODER |= (1 << (YELLOW_LED_PIN * 2));
    GB_LED_PORT->MODER |= (1 << (GB_LED_PIN * 2));
    BUZZER_PORT->MODER |= (1 << (BUZZER_PIN * 2));

    // 3. Configure Input Pins (Buttons, Intrusion Sensor)
    // S1 (PC13) is already input by default. No pull-up/down needed on board.
    S2_PORT->PUPDR |= (1 << (S2_PIN * 2)); // Pull-up
    S3_PORT->PUPDR |= (1 << (S3_PIN * 2)); // Pull-up
    INTRUSION_PORT->PUPDR |= (1 << (INTRUSION_PIN * 2)); // Pull-up

    // 4. Configure Analog Pins (Sensors)
    TEMP_SENSOR_PORT->MODER |= (3 << (TEMP_SENSOR_PIN * 2));
    POT_PORT->MODER |= (3 << (POT_PIN * 2));

    // 5. Configure ADC
    ADC1->CR2 |= ADC_CR2_ADON; // Turn on ADC
    ADC1->SMPR2 |= (7 << (TEMP_SENSOR_PIN * 3)) | (7 << (POT_PIN * 3)); // Max sample time for stability

    // 6. Enable FPU for floating point math
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

// --- Input Functions Implementation ---
float HAL_GetTemperature(void) {
    uint16_t adc_val = read_adc_channel(TEMP_SENSOR_PIN);
    float v_out = (adc_val * ADC_VREF) / ADC_MAX_VAL;
    float r_ntc = (v_out * NTC_PULLUP_R) / (ADC_VREF - v_out);

    float temp_k = (NTC_BETA * NTC_T0) / (NTC_T0 * logf(r_ntc / NTC_R0) + NTC_BETA);
    return temp_k - 273.15f; // Convert Kelvin to Celsius
}

uint16_t HAL_GetPotValue(void) {
    return read_adc_channel(POT_PIN);
}

bool HAL_IsIntrusionDetected(void) {
    // Assuming active-low sensor (goes to 0 when intrusion detected)
    return !(INTRUSION_PORT->IDR & (1 << INTRUSION_PIN));
}

// Simple debouncing by checking state change
bool check_button_press(GPIO_TypeDef* port, uint16_t pin, bool active_low) {
    static uint32_t last_states = 0xFFFFFFFF; // Assume all buttons released initially
    uint32_t button_mask = (port == GPIOC && pin == 13) ? 0 : 1; // Unique ID for each button can be improved

    bool current_state = (port->IDR & (1 << pin));
    if (active_low) current_state = !current_state;

    bool pressed = false;
    if (current_state && !(last_states & button_mask)) {
        pressed = true; // Was released, now pressed
    }

    if (current_state) last_states |= button_mask;
    else last_states &= ~button_mask;

    if(pressed) HAL_Delay(50); // Simple debounce delay

    return pressed;
}

bool HAL_IsS1Pressed(void) {
    // PC13 is active-low
    return !(S1_PORT->IDR & (1 << S1_PIN)); // Using simple check for main button
}

bool HAL_IsS2Pressed(void) {
    // Assuming active-low with pull-up
    return !(S2_PORT->IDR & (1 << S2_PIN));
}

bool HAL_IsS3Pressed(void) {
    // Assuming active-low with pull-up
    return !(S3_PORT->IDR & (1 << S3_PIN));
}

bool HAL_IsYellowLED_On(void) {
    return (YELLOW_LED_PORT->ODR & (1 << YELLOW_LED_PIN));
}

// --- Output Functions Implementation ---
void HAL_SetRedLED(bool state) {
    if (state) RED_LED_PORT->BSRR = (1 << RED_LED_PIN);
    else RED_LED_PORT->BSRR = (1 << (RED_LED_PIN + 16));
}

void HAL_SetYellowLED(bool state) {
    if (state) YELLOW_LED_PORT->BSRR = (1 << YELLOW_LED_PIN);
    else YELLOW_LED_PORT->BSRR = (1 << (YELLOW_LED_PIN + 16));
}

void HAL_SetGB_LED(bool state) {
    if (state) GB_LED_PORT->BSRR = (1 << GB_LED_PIN);
    else GB_LED_PORT->BSRR = (1 << (GB_LED_PIN + 16));
}

void HAL_SetBuzzer(bool state) {
//    if (state) BUZZER_PORT->BSRR = (1 << BUZZER_PIN);
//    else BUZZER_PORT->BSRR = (1 << (BUZZER_PIN + 16));
	 if (state) {
	        BUZZER_PORT->BSRR = (1 << BUZZER_PIN);
	        HAL_SetYellowLED(true); // <--- เพิ่มบรรทัดนี้
	    } else {
	        BUZZER_PORT->BSRR = (1 << (BUZZER_PIN + 16));
	        HAL_SetYellowLED(false); // <--- เพิ่มบรรทัดนี้
	    }
}

// --- Combined Alarm Functions ---
void HAL_ActivateFireAlarm(void) {
    HAL_SetRedLED(true);
    HAL_SetBuzzer(true);
}

void HAL_ActivateIntruderAlarm(void) {
    HAL_SetRedLED(true); // Or flash all LEDs
    HAL_SetYellowLED(true);
    HAL_SetGB_LED(true);
    HAL_SetBuzzer(true);
}

void HAL_DeactivateAllAlarms(void) {
    HAL_SetRedLED(false);
    HAL_SetYellowLED(false);
    HAL_SetGB_LED(false);
    HAL_SetBuzzer(false);
}

// --- Utility ---
void HAL_Delay(uint32_t ms) {
    // Rough delay for 16MHz clock
    volatile uint32_t count = ms * 1600;
    while(count--);
}
