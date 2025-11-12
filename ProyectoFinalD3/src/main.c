#include "samples.h"
#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"

#define FREQ_MUESTREO 16000   // Frecuencia de muestreo DAC
#define PCLK_TIMER0    25000000 // Frecuencia periférica T0 (ajustar según tu config)

volatile uint32_t idx = 0;
volatile uint16_t vol_A = 2048; // Valor inicial medio (ganancia 0.5 aprox)

void cfgDAC(void);
void cfgTMR(void);
void cfgGPIO(void);
void cfgADC(void);



int main(void) {
    cfgGPIO();
    cfgDAC();
    cfgADC();
    cfgTMR();

    while (1) {
        // Nada, el control lo hace el TIMER y el ADC
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

    // --- DAC output ---
    int32_t muestra = (int32_t)sampleS5[idx] - 128;
    int32_t salida = (muestra * vol_A) >> 12;
    salida += 128;
    if (salida > 255) salida = 255;
    if (salida < 0) salida = 0;
    uint16_t val10b = salida << 2;
    DAC_UpdateValue(val10b);

    idx++;
    if (idx >= SAMPLE_S5_LEN) idx = 0;

    // --- ADC trigger cada 500 muestras
    if (++contador_adc >= 500) {
        contador_adc = 0;
        ADC_StartCmd(ADC_START_NOW);
    }

    TIM_ClearIntPending(LPC_TIM0, TIM_MR1_INT);
}

void ADC_IRQHandler(void) {
    if (ADC_ChannelGetStatus( 0, 1)) {
        vol_A = ADC_ChannelGetData( 0);
    }
}

