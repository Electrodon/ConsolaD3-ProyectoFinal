#include "samples.h"
#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"

#define FREQ_MUESTREO 16000   // Frecuencia de muestreo DAC
#define PCLK_TIMER0    25000000 // Frecuencia periférica T0 (ajustar según tu config)
#define PORT_0 (uint8_t)0
#define PIN_1 (uint32_t)(1<<1)
#define NUM_SAMPLES 2

typedef struct {
    const uint8_t *data;      // puntero al arreglo en Flash
    uint32_t length;          // cantidad de muestras
    uint32_t index;           // índice actual
    uint8_t active;           // 1 si está reproduciendo
} SamplePlayer;

SamplePlayer players[NUM_SAMPLES];

volatile uint32_t idx = 0;
volatile uint16_t vol_A = 2048; // Valor inicial medio (ganancia 0.5 aprox)

void cfgDAC(void);
void cfgTMR(void);
void cfgGPIO(void);
void cfgADC(void);
void initSamples(void);


int main(void) {
    cfgGPIO();
    cfgDAC();
    cfgADC();
    cfgTMR();
    initSamples();

    while (1) {

    }
    return 0;
}



void cfgGPIO(void) {
    // DAC (P0.26)
    PINSEL_CFG_Type cfgPinDac;
    cfgPinDac.portNum = 0;
    cfgPinDac.pinNum = 26;
    cfgPinDac.funcNum = 2;
    cfgPinDac.pinMode = PINSEL_TRISTATE;
    cfgPinDac.openDrain = PINSEL_OD_NORMAL;
    PINSEL_ConfigPin(&cfgPinDac);

    // ADC0.0 (P0.23)
    PINSEL_CFG_Type cfgADC0;
    cfgADC0.portNum = 0;
    cfgADC0.pinNum = 23;
    cfgADC0.funcNum = 1;
    cfgADC0.pinMode = PINSEL_TRISTATE;
    cfgADC0.openDrain = PINSEL_OD_NORMAL;
    PINSEL_ConfigPin(&cfgADC0);
}

void cfgDAC(void) {
    DAC_Init();
}

void cfgADC(void) {
    ADC_Init(200000);             // Frecuencia de muestreo del ADC = 200 kHz
    ADC_ChannelCmd(0, ENABLE);    // Canal 0 (P0.23)
    ADC_IntConfig(ADC_ADINTEN0, ENABLE);
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1);
}

void cfgTMR(void) {
    TIM_TIMERCFG_Type cfgTimer;
    TIM_MATCHCFG_Type cfgMatch;

    cfgTimer.prescaleOption = TIM_USVAL;
    cfgTimer.prescaleValue = 1; // 1 µs

    cfgMatch.matchChannel = 1;
    cfgMatch.matchValue = (1000000 / FREQ_MUESTREO); // 62.5 µs
    cfgMatch.intOnMatch = ENABLE;
    cfgMatch.stopOnMatch = DISABLE;
    cfgMatch.resetOnMatch = ENABLE;
    cfgMatch.extMatchOutputType = TIM_NOTHING;

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfgTimer);
    TIM_ConfigMatch(LPC_TIM0, &cfgMatch);
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

void TIMER0_IRQHandler(void) {
    static uint16_t contador_adc = 0;
    int32_t mix = 0;
    uint8_t activos = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        if (players[i].active) {
            int32_t muestra = (int32_t)players[i].data[players[i].index] - 128;
            int32_t salida = (muestra * vol_A) >> 12;
            mix += salida;

            // avanzar índice
            if (++players[i].index >= players[i].length)
                players[i].index = 0;
            activos++;
        }
    }

    if (activos > 0) mix /= activos;  // normalizar mezcla
    if(activos == NUM_SAMPLES) activos = 0;
    mix += 128;
    if (mix > 255) mix = 255;
    if (mix < 0) mix = 0;

    uint16_t val10bit = (uint16_t)(mix << 2);
    DAC_UpdateValue(val10bit);

    // trigger ADC cada 500 muestras (~32 Hz)
    if (++contador_adc >= 500) {
        contador_adc = 0;
        ADC_StartCmd(ADC_START_NOW);
    }

    TIM_ClearIntPending(LPC_TIM0, TIM_MR1_INT);
}


void ADC_IRQHandler(void) {
    if (ADC_ChannelGetStatus(0,1)) {
        vol_A = ADC_ChannelGetData(0);
    }
}

void initSamples(void) {
    players[0].data = sampleS1;
    players[0].length = SAMPLE_S1_LEN;
    players[0].index = 0;
    players[0].active = 0;

    players[1].data = sampleS5;
    players[1].length = SAMPLE_S5_LEN;
    players[1].index = 0;
    players[1].active = 0;
    return;
}
