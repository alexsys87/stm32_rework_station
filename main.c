#include "stm32f1xx.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

// =======================================================
// НАСТРОЙКИ СИСТЕМЫ И ПИД-РЕГУЛЯТОРА
// =======================================================
#define MAX_TEMP                480.0f  // Максимальная температура
#define MIN_TEMP                50.0f   // Минимальная уставка температуры
#define COOLING_TARGET          50.0f   // Целевая температура остывания
#define COOLING_HYSTERESIS      5.0f    // Гистерезис отключения насоса
#define PUMP_MIN_POWER          25      // Минимальная мощность насоса (%)

// Настройки АЦП и DMA-буфера (режим Ping-Pong)
#define ADC_SAMPLES_PER_CH      16
#define NUM_ADC_CHANNELS        4
#define DMA_BUFFER_SIZE         (NUM_ADC_CHANNELS * ADC_SAMPLES_PER_CH * 2)

// ПИД-регулятор
#define PID_DT                  0.05f   // 50 мс период дискретизации
#define PID_KP                  2.5f
#define PID_KI                  3.0f    
#define PID_KD                  0.004f  

// =======================================================
// ПИНЫ И ПОРТЫ
// =======================================================
#define MAX7219_CS_PIN          GPIO_PIN_12
#define MAX7219_CS_PORT         GPIOB
#define LED_PIN                 GPIO_PIN_13
#define LED_GPIO_PORT           GPIOC
#define TRIAC_PIN               GPIO_PIN_1
#define TRIAC_GPIO_PORT         GPIOB
#define PUMP_TRIAC_PIN          GPIO_PIN_10
#define PUMP_TRIAC_GPIO_PORT    GPIOB
#define CONTROL_INPUT_PIN       GPIO_PIN_4
#define CONTROL_INPUT_PORT      GPIOA
#define ZERO_CROSS_PIN          GPIO_PIN_6
#define ZERO_CROSS_PORT         GPIOA

// Регистры MAX7219
#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DIGIT1      0x02
#define MAX7219_REG_DIGIT2      0x03
#define MAX7219_REG_DIGIT3      0x04
#define MAX7219_REG_DIGIT4      0x05
#define MAX7219_REG_DIGIT5      0x06
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0A
#define MAX7219_REG_SCANLIMIT   0x0B
#define MAX7219_REG_SHUTDOWN    0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

#define SEG_EMPTY               10
#define SEG_C                   11
#define SEG_O                   12
#define SEG_L                   13
#define SEG_E                   14
#define SEG_R                   15

// =======================================================
// ПИНЫ И ПОРТЫ
// =======================================================
 #define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
 #define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
 #define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
 #define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
 #define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
 #define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
 #define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
 #define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
 #define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
 #define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
 #define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
 #define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
 #define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
 #define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
 #define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
 #define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */

// =======================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =======================================================
volatile uint32_t sys_tick_ms = 0;
volatile uint16_t ac_period = 10000;
volatile uint16_t last_zc_time = 0;

volatile uint8_t power_percent = 0;
volatile uint8_t pump_power_percent = 0;
volatile float pump_target_percent = 0.0f;

volatile float current_temperature = 0.0f;
volatile float cold_junction_temp = 0.0f;

volatile uint8_t cooling_mode_active = 0;
volatile uint8_t is_working = 0;
volatile uint8_t heater_fault_detected = 0;

// Буферы DMA и АЦП
volatile uint16_t adc_dma_buffer[DMA_BUFFER_SIZE];
volatile uint16_t safe_adc_values[NUM_ADC_CHANNELS];

typedef struct {
    float setpoint;
    float integral;
    float prev_error;
} PID_Controller;

PID_Controller pid = {0};

const uint8_t segment_font[] = {
    0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, // Цифры 0-9
    0x00, // 10: пусто
    0x4E, // 11: C
    0x7E, // 12: O (выглядит как 0)
    0x0E, // 13: L
    0x4F, // 14: E
    0x05  // 15: r
};

// =======================================================
// УТИЛИТЫ И ЗАДЕРЖКИ
// =======================================================
void Delay_us(uint32_t us) {
    uint16_t start = TIM2->CNT;
    while ((uint16_t)(TIM2->CNT - start) < us);
}

// =======================================================
// ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ
// =======================================================
void System_Clocks_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_SPI2EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
}

void GPIO_Init(void) {
    // Аналоговые входы: PA0, PA1, PA2, PA3
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0 | GPIO_CRL_CNF1 | GPIO_CRL_MODE1 |
                    GPIO_CRL_CNF2 | GPIO_CRL_MODE2 | GPIO_CRL_CNF3 | GPIO_CRL_MODE3);

    // PA4 - Контрольный вход (подтяжка к питанию)
    GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4);
    GPIOA->CRL |= GPIO_CRL_CNF4_1; 
    GPIOA->BSRR = CONTROL_INPUT_PIN;

    // PA6 - Вход Zero Cross (Внешнее прерывание, подтяжка к питанию)
    GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
    GPIOA->CRL |= GPIO_CRL_CNF6_1; 
    GPIOA->BSRR = ZERO_CROSS_PIN;

    // Выходы на симисторы: PB1, PB10
    GPIOB->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
    GPIOB->CRL |= (GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);
    GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
    GPIOB->CRH |= (GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1);

    // Аппаратный SPI2 и MAX7219 CS
    GPIOB->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12 | GPIO_CRH_CNF13 | GPIO_CRH_MODE13 |
                    GPIO_CRH_CNF14 | GPIO_CRH_MODE14 | GPIO_CRH_CNF15 | GPIO_CRH_MODE15);
    GPIOB->CRH |= (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1); // CS PB12
    GPIOB->CRH |= (GPIO_CRH_CNF13_1 | GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0); // SCK
    GPIOB->CRH |= GPIO_CRH_CNF14_1; // MISO
    GPIOB->CRH |= (GPIO_CRH_CNF15_1 | GPIO_CRH_MODE15_1 | GPIO_CRH_MODE15_0); // MOSI

    // Встроенный светодиод PC13
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_1;

    LED_GPIO_PORT->BSRR = LED_PIN;
    TRIAC_GPIO_PORT->BRR = TRIAC_PIN;
    PUMP_TRIAC_GPIO_PORT->BRR = PUMP_TRIAC_PIN;
    MAX7219_CS_PORT->BSRR = MAX7219_CS_PIN;
}

void ADC_DMA_Init(void) {
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_dma_buffer;
    DMA1_Channel1->CNDTR = DMA_BUFFER_SIZE;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | 
                         DMA_CCR_CIRC | DMA_CCR_HTIE | DMA_CCR_TCIE;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA;
    ADC1->SMPR2 = ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3;
    ADC1->SQR1 = ((NUM_ADC_CHANNELS - 1) << 20);
    ADC1->SQR3 = (0 << 0) | (1 << 5) | (2 << 10) | (3 << 15);

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while(ADC1->CR2 & ADC_CR2_RSTCAL);
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);

    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void EXTI_Init(void) {
    AFIO->EXTICR[1] &= ~AFIO_EXTICR2_EXTI6;
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PA;
    EXTI->IMR |= EXTI_IMR_MR6;
    EXTI->FTSR |= EXTI_FTSR_TR6; // Срабатывание по спаду импульса от оптопары
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Timers_Init(void) {
    SysTick_Config(SystemCoreClock / 1000); // 1 мс

    // TIM2 (1 МГц) - для микросекундных задержек и периода Zero-Cross
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;

    // TIM3 (1 МГц) - формирование импульсов на симисторы
    TIM3->PSC = 72 - 1;
    TIM3->ARR = 0xFFFF;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
}

void SPI_Init(void) {
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE | SPI_CR1_BR_1;
}

void MAX7219_WriteRegister(uint8_t reg, uint8_t data) {
    // Критическая секция: защищаем SPI транзакцию (занимает ~2 мкс)
    __disable_irq(); 
    MAX7219_CS_PORT->BRR = MAX7219_CS_PIN;
    
    SPI2->DR = reg;
    while (!(SPI2->SR & SPI_SR_TXE));
    while (SPI2->SR & SPI_SR_BSY);
    
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_TXE));
    while (SPI2->SR & SPI_SR_BSY);
    
    MAX7219_CS_PORT->BSRR = MAX7219_CS_PIN;
    __enable_irq();
}

void MAX7219_Init(void) {
    MAX7219_WriteRegister(MAX7219_REG_SHUTDOWN, 0x01);
    MAX7219_WriteRegister(MAX7219_REG_DISPLAYTEST, 0x00);
    MAX7219_WriteRegister(MAX7219_REG_SCANLIMIT, 0x05);
    MAX7219_WriteRegister(MAX7219_REG_DECODEMODE, 0x00);
    MAX7219_WriteRegister(MAX7219_REG_INTENSITY, 0x08);
}

void IWDG_Init(void) {
    IWDG->KR = 0x5555;
    IWDG->PR = 3;       
    IWDG->RLR = 500;    // Сторожевой таймер на ~400 мс
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
}

// =======================================================
// ПРЕРЫВАНИЯ (Критичные ко времени)
// =======================================================
void SysTick_Handler(void) {
    sys_tick_ms++;
}

// Ping-Pong обработка DMA: усредняем ту половину массива, куда АЦП сейчас НЕ пишет
void DMA1_Channel1_IRQHandler(void) {
    uint32_t offset = 0;
    
    if (DMA1->ISR & DMA_ISR_HTIF1) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        offset = 0; // АЦП заполняет 2-ю половину, безопасно читаем 1-ю
    } else if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        offset = NUM_ADC_CHANNELS * ADC_SAMPLES_PER_CH; // Читаем 2-ю
    }

    uint32_t sum[NUM_ADC_CHANNELS] = {0};
    for (int i = 0; i < ADC_SAMPLES_PER_CH; i++) {
        sum[0] += adc_dma_buffer[offset + i * NUM_ADC_CHANNELS + 0];
        sum[1] += adc_dma_buffer[offset + i * NUM_ADC_CHANNELS + 1];
        sum[2] += adc_dma_buffer[offset + i * NUM_ADC_CHANNELS + 2];
        sum[3] += adc_dma_buffer[offset + i * NUM_ADC_CHANNELS + 3];
    }

    safe_adc_values[0] = sum[0] / ADC_SAMPLES_PER_CH;
    safe_adc_values[1] = sum[1] / ADC_SAMPLES_PER_CH;
    safe_adc_values[2] = sum[2] / ADC_SAMPLES_PER_CH;
    safe_adc_values[3] = sum[3] / ADC_SAMPLES_PER_CH;
}

// Прерывание перехода через ноль (Zero Cross)
void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR6) {
        EXTI->PR = EXTI_PR_PR6; 

        uint16_t now = TIM2->CNT;
        uint16_t period = now - last_zc_time;
        last_zc_time = now;

        if (period > 8000 && period < 12000) ac_period = period;
        TIM3->CNT = 0; // Синхронизируем таймер импульсов

        // Нагреватель
        if (power_percent > 0 && power_percent <= 100) {
            uint16_t delay = ac_period - (ac_period * power_percent / 100);
            if (delay < 100) delay = 100; 
            TIM3->CCR1 = delay;         // Момент открытия симистора
            TIM3->CCR3 = delay + 50;    // Момент закрытия (импульс 50 мкс)
            TIM3->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC3IF);
            TIM3->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC3IE);
        } else {
            TIM3->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC3IE);
            TRIAC_GPIO_PORT->BRR = TRIAC_PIN; 
        }

        // Насос
        if (pump_power_percent > 0 && pump_power_percent <= 100) {
            uint16_t delay = ac_period - (ac_period * pump_power_percent / 100);
            if (delay < 100) delay = 100;
            TIM3->CCR2 = delay;         
            TIM3->CCR4 = delay + 200;   // Импульс 200 мкс (для индуктивной нагрузки)
            TIM3->SR &= ~(TIM_SR_CC2IF | TIM_SR_CC4IF);
            TIM3->DIER |= (TIM_DIER_CC2IE | TIM_DIER_CC4IE);
        } else {
            TIM3->DIER &= ~(TIM_DIER_CC2IE | TIM_DIER_CC4IE);
            PUMP_TRIAC_GPIO_PORT->BRR = PUMP_TRIAC_PIN;
        }
    }
}

// Прерывание формирования импульсов на симисторы
void TIM3_IRQHandler(void) {
    if ((TIM3->SR & TIM_SR_CC1IF) && (TIM3->DIER & TIM_DIER_CC1IE)) {
        TIM3->SR &= ~TIM_SR_CC1IF;
        TRIAC_GPIO_PORT->BSRR = TRIAC_PIN;
    }
    if ((TIM3->SR & TIM_SR_CC3IF) && (TIM3->DIER & TIM_DIER_CC3IE)) {
        TIM3->SR &= ~TIM_SR_CC3IF;
        TRIAC_GPIO_PORT->BRR = TRIAC_PIN;
        TIM3->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC3IE);
    }
    if ((TIM3->SR & TIM_SR_CC2IF) && (TIM3->DIER & TIM_DIER_CC2IE)) {
        TIM3->SR &= ~TIM_SR_CC2IF;
        PUMP_TRIAC_GPIO_PORT->BSRR = PUMP_TRIAC_PIN;
    }
    if ((TIM3->SR & TIM_SR_CC4IF) && (TIM3->DIER & TIM_DIER_CC4IE)) {
        TIM3->SR &= ~TIM_SR_CC4IF;
        PUMP_TRIAC_GPIO_PORT->BRR = PUMP_TRIAC_PIN;
        TIM3->DIER &= ~(TIM_DIER_CC2IE | TIM_DIER_CC4IE);
    }
}

// =======================================================
// ЛОГИКА MAIN
// =======================================================
void Read_Sensors(void) {
    uint16_t local_adc[NUM_ADC_CHANNELS];
    
    // Атомарное чтение буфера АЦП
    __disable_irq();
    local_adc[0] = safe_adc_values[0];
    local_adc[1] = safe_adc_values[1];
    local_adc[2] = safe_adc_values[2];
    local_adc[3] = safe_adc_values[3];
    __enable_irq();

    // Расчет температуры холодного спая (NTC)
    float voltage_ntc = (local_adc[3] * 3.3f) / 4095.0f;
    if (voltage_ntc > 0.1f && voltage_ntc < 3.2f) {
        float ntc_resistance = (10000.0f * voltage_ntc) / (3.3f - voltage_ntc);
        float steinhart = logf(ntc_resistance / 10000.0f) / 3950.0f + (1.0f / 298.15f);
        cold_junction_temp = (1.0f / steinhart) - 273.15f;
    } else {
        cold_junction_temp = 25.0f; 
    }

    // Расчет температуры термопары
    float voltage_tc = (local_adc[2] * 3300.0f) / 4095.0f;
    float hot_temp = 24.5f * voltage_tc + 0.045f * voltage_tc * voltage_tc; 
    current_temperature = hot_temp + cold_junction_temp;
    
    // Уставки
    pid.setpoint = (local_adc[1] * (MAX_TEMP - MIN_TEMP) / 4095.0f) + MIN_TEMP;
    pump_target_percent = (local_adc[0] * (100.0f - PUMP_MIN_POWER) / 4095.0f) + PUMP_MIN_POWER;
}

void Compute_Controls(void) {
    // Антидребезг (Debounce) кнопки управления герконом/выключателем
    static uint8_t switch_history = 0xFF;
    uint8_t current_btn_state = (GPIOA->IDR & CONTROL_INPUT_PIN) ? 1 : 0;
    
    switch_history = (switch_history << 1) | current_btn_state;
    if ((switch_history & 0x0F) == 0x00) { 
        is_working = 1; 
    } else if ((switch_history & 0x0F) == 0x0F) {
        is_working = 0; 
    }

    if (is_working && !heater_fault_detected) {
        // ШТАТНАЯ РАБОТА
        cooling_mode_active = 0;
        
        float error = pid.setpoint - current_temperature;
        
        pid.integral += PID_KI * error * PID_DT;
        if (pid.integral > 100.0f) pid.integral = 100.0f;
        if (pid.integral < 0.0f)   pid.integral = 0.0f;
        
        float derivative = PID_KD * (error - pid.prev_error) / PID_DT;
        pid.prev_error = error;
        
        float output = (PID_KP * error) + pid.integral + derivative;
        if(output > 100.0f) output = 100.0f;
        if(output < 0.0f)   output = 0.0f;

        power_percent = (uint8_t)output;
        pump_power_percent = (uint8_t)(pump_target_percent + 0.5f);
        
    } else {
        // РЕЖИМ ОСТАНОВКИ ИЛИ АВАРИИ
        cooling_mode_active = 1;
        power_percent = 0; // Обязательное отключение нагревателя
        pid.integral = 0;
        pid.prev_error = 0;

        if (heater_fault_detected) {
            // АВАРИЯ: Максимальная мощность помпы для спасения фена
            pump_power_percent = 100;
        } else if (current_temperature > (COOLING_TARGET + COOLING_HYSTERESIS)) {
            // ШТАТНОЕ ОХЛАЖДЕНИЕ: Продувка
            pump_power_percent = 80;
        } else {
            // ОСТЫЛ: Отключаем помпу
            pump_power_percent = 0;
        }
    }
}

void Refresh_Display(void) {
    // 1. ИНДИКАЦИЯ АВАРИИ (Перегрев)
    if (heater_fault_detected) {
        if ((sys_tick_ms / 500) % 2 == 0) { // Мигаем надписью Err каждые 500 мс
            MAX7219_WriteRegister(MAX7219_REG_DIGIT5, segment_font[SEG_E]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT4, segment_font[SEG_R]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT3, segment_font[SEG_R]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT2, segment_font[SEG_EMPTY]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT1, segment_font[SEG_EMPTY]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT0, segment_font[SEG_EMPTY]);
            return;
        }
    } 
    // 2. ИНДИКАЦИЯ ОХЛАЖДЕНИЯ (Штатный)
    else if (cooling_mode_active) {
        if (current_temperature <= (COOLING_TARGET - COOLING_HYSTERESIS)) {
            // Полностью остыл -> статика
            MAX7219_WriteRegister(MAX7219_REG_DIGIT5, segment_font[SEG_C]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT4, segment_font[SEG_O]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT3, segment_font[SEG_L]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT2, segment_font[SEG_EMPTY]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT1, segment_font[SEG_EMPTY]);
            MAX7219_WriteRegister(MAX7219_REG_DIGIT0, segment_font[SEG_EMPTY]);
            return;
        } else {
            // Процесс остывания -> мигаем COL / Температура
            if ((sys_tick_ms / 1000) % 2 == 0) {
                MAX7219_WriteRegister(MAX7219_REG_DIGIT5, segment_font[SEG_C]);
                MAX7219_WriteRegister(MAX7219_REG_DIGIT4, segment_font[SEG_O]);
                MAX7219_WriteRegister(MAX7219_REG_DIGIT3, segment_font[SEG_L]);
                MAX7219_WriteRegister(MAX7219_REG_DIGIT2, segment_font[SEG_EMPTY]);
                MAX7219_WriteRegister(MAX7219_REG_DIGIT1, segment_font[SEG_EMPTY]);
                MAX7219_WriteRegister(MAX7219_REG_DIGIT0, segment_font[SEG_EMPTY]);
                return;
            }
        }
    }

    // 3. СТАНДАРТНЫЙ ВЫВОД (Температура и Уставка)
    uint16_t real = (uint16_t)current_temperature;
    if(real > 999) real = 999;
    MAX7219_WriteRegister(MAX7219_REG_DIGIT5, (real >= 100) ? segment_font[real / 100] : segment_font[SEG_EMPTY]);
    MAX7219_WriteRegister(MAX7219_REG_DIGIT4, (real >= 10) ? segment_font[(real / 10) % 10] : segment_font[SEG_EMPTY]);
    MAX7219_WriteRegister(MAX7219_REG_DIGIT3, segment_font[real % 10] | 0x80); // Десятичная точка

    uint16_t set = (uint16_t)pid.setpoint;
    if(set > 999) set = 999;
    MAX7219_WriteRegister(MAX7219_REG_DIGIT2, (set >= 100) ? segment_font[set / 100] : segment_font[SEG_EMPTY]);
    MAX7219_WriteRegister(MAX7219_REG_DIGIT1, (set >= 10) ? segment_font[(set / 10) % 10] : segment_font[SEG_EMPTY]);
    MAX7219_WriteRegister(MAX7219_REG_DIGIT0, segment_font[set % 10]);
}

// =======================================================
// MAIN
// =======================================================
int main(void) {
    SystemInit();
    System_Clocks_Init();
    
    // Инициализация периферии
    GPIO_Init();
    ADC_DMA_Init();
    EXTI_Init();
    Timers_Init();
    SPI_Init();

    // ЯВНАЯ НАСТРОЙКА ПРИОРИТЕТОВ ПРЕРЫВАНИЙ (Industrial Approach)
    NVIC_SetPriority(EXTI9_5_IRQn, 0);       // Детектор нуля (высший приоритет)
    NVIC_SetPriority(TIM3_IRQn, 1);          // Управление симистором (тоже критично)
    NVIC_SetPriority(DMA1_Channel1_IRQn, 2); // Сбор данных с АЦП
    NVIC_SetPriority(SysTick_IRQn, 15);      // Системный таймер отсчета (низший)
    
    // Стабилизация питания дисплея
    Delay_us(50000); 
    MAX7219_Init();
    IWDG_Init();

    uint32_t last_calc_time = 0;
    uint32_t last_disp_time = 0;
    uint32_t last_led_time = 0;

    while(1) {
        uint32_t current_ms = sys_tick_ms; 

        // Расчет логики каждые 50 мс
        if (current_ms - last_calc_time >= (uint32_t)(PID_DT * 1000)) {
            last_calc_time = current_ms;
            
            Read_Sensors();
            
            // Программная защита от перегрева
            if(current_temperature > 510.0f) {
                heater_fault_detected = 1;
            } else if(heater_fault_detected && current_temperature < 490.0f) {
                heater_fault_detected = 0;
            }

            Compute_Controls();
        }

        // Обновление дисплея каждые 100 мс
        if (current_ms - last_disp_time >= 100) {
            last_disp_time = current_ms;
            Refresh_Display();
        }

        // Статусный диод каждые 1 секунду
        if (current_ms - last_led_time >= 1000) {
            last_led_time = current_ms;
            if(is_working && !heater_fault_detected) {
                LED_GPIO_PORT->ODR ^= LED_PIN; // Мигает в нормальном режиме
            } else {
                LED_GPIO_PORT->BSRR = LED_PIN; // Погашен при спячке или аварии
            }
        }

        // Кормим WatchDog
        IWDG->KR = 0xAAAA;
    }
}