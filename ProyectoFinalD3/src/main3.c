#include "samples.h"
#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_uart.h"
#include <string.h>


#define FREQ_MUESTREO 16000   // Frecuencia de muestreo DAC
#define PCLK_TIMER0    25000000 // Frecuencia periférica T0 (ajustar según tu config)
#define PORT_0 (uint8_t)0
#define PIN_1 (uint32_t)(1<<1)
#define NUM_SAMPLES 6
#define OUTPUT (uint8_t)1
#define INPUT  (uint8_t)0
#define PIN_22 (1U << 22)


typedef struct {
    const uint8_t *data;      // puntero al arreglo en Flash
    uint32_t length;          // cantidad de muestras
    volatile uint32_t index;           // índice actual
    volatile uint8_t active;           // 1 si está reproduciendo
} SamplePlayer;

SamplePlayer players[NUM_SAMPLES];

volatile uint32_t idx = 0;
volatile uint16_t vol_A = 2048; // Valor inicial medio (ganancia 0.5 aprox)

volatile uint8_t pianoMode = 0;

void cfgDAC(void);
void cfgTMR(void);
void cfgGPIO(void);
void cfgADC(void);
void cfgUART(void);
void initSamples(void);
void sendWelcomeMessage(void);

int main(void) {
    cfgGPIO();
    cfgDAC();
    cfgADC();
    cfgTMR();
    cfgUART();
    initSamples();
    sendWelcomeMessage();
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

    PINSEL_CFG_Type cfgPinLED, cfgPinRXD0, cfgPinTXD0;

    // --- LED en P0.22 ---
    cfgPinLED.portNum   = PINSEL_PORT_0;
    cfgPinLED.pinNum    = PINSEL_PIN_22;
    cfgPinLED.funcNum   = PINSEL_FUNC_0;
    cfgPinLED.pinMode   = PINSEL_PULLUP;
    cfgPinLED.openDrain = PINSEL_OD_NORMAL;
    PINSEL_ConfigPin(&cfgPinLED);

    // --- RXD0 (P0.3) ---
    cfgPinRXD0.portNum   = PINSEL_PORT_0;
    cfgPinRXD0.pinNum    = PINSEL_PIN_3;
    cfgPinRXD0.funcNum   = PINSEL_FUNC_1;
    cfgPinRXD0.pinMode   = PINSEL_PULLUP;
    cfgPinRXD0.openDrain = PINSEL_OD_NORMAL;
    PINSEL_ConfigPin(&cfgPinRXD0);

    // --- TXD0 (P0.2) ---
    cfgPinTXD0.portNum   = PINSEL_PORT_0;
    cfgPinTXD0.pinNum    = PINSEL_PIN_2;
    cfgPinTXD0.funcNum   = PINSEL_FUNC_1;
    cfgPinTXD0.pinMode   = PINSEL_PULLUP;
    cfgPinTXD0.openDrain = PINSEL_OD_NORMAL;
    PINSEL_ConfigPin(&cfgPinTXD0);

    GPIO_SetDir(PORT_0, PIN_22, OUTPUT);
    LPC_GPIO0->FIOSET|=(PIN_22);
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

    cfgTimer.prescaleOption =  TIM_USVAL;
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
                if (pianoMode){players[i].active = 0;}
            activos++;
        }
    }

    //    Si hay 2 samples sonando, dividimos por 2 para evitar saturación.
    //    Si hay 1 sample, 'mix' queda intacto (volumen completo).
    if (activos > 1) {
        mix = mix >> 1; // Divide por 2 (equivale a mix /= 2)
    }

    //    Convertir a rango SIN SIGNO (0-255) para el DAC
    mix += 128;

    //    Saturación (seguridad)
    //    Aseguramos que el valor esté en el rango de 8 bits
    if (mix > 255) mix = 255;
    if (mix < 0) mix = 0;

    //    Convertir 8 bits (0-255) a 10 bits (0-1023) para el DAC
    uint16_t val10bit = (uint16_t)(mix << 2);
    DAC_UpdateValue(val10bit);

    if (++contador_adc >= 500) {
        contador_adc = 0;
        ADC_StartCmd(ADC_START_NOW); // (Asumo LPC_ADC)
    }


    TIM_ClearIntPending(LPC_TIM0, TIM_MR1_INT);
}

void cfgUART(void)
{
	LPC_SC->PCONP |= (1 << 3);

    UART_CFG_Type cfgUART0;
    UART_FIFO_CFG_Type cfgUART0FIFO;

    UART_ConfigStructInit(&cfgUART0);
    cfgUART0.Baud_rate = 115200;

    UART_Init((LPC_UART_TypeDef *)LPC_UART0, &cfgUART0);
    UART_FIFOConfigStructInit(&cfgUART0FIFO);
    UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &cfgUART0FIFO);
    UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RBR, ENABLE); // Habilita interrupción por "Receive Data Ready"
    UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);

    NVIC_EnableIRQ(UART0_IRQn);
}

void UART0_IRQHandler(void)
{
    uint8_t receivedData;
    uint32_t iir_value;

    // Leemos el registro de identificación de interrupción para saber qué la causó
    iir_value = UART_GetIntId((LPC_UART_TypeDef *)LPC_UART0);

    // Verificamos si la interrupción fue por "Receive Data Ready" (RDA)
    if ((iir_value & UART_IIR_INTID_RDA) || (iir_value & UART_IIR_INTID_CTI))
    {
        receivedData = UART_ReceiveByte((LPC_UART_TypeDef *)LPC_UART0);

       // UART_SendByte((LPC_UART_TypeDef *)LPC_UART0, receivedData);

        switch(receivedData)
        {
	    case '1': players[0].active = (players[0].active +1)%2;  // Activo/Desactivo Sample 1
            if(players[0].active) players[0].index = 0;
	    break;
        case '2': players[1].active = (players[1].active+1)%2;  // Activo/Desactivo Sample 2
            if(players[1].active) players[1].index = 0;
	    break;
        case '3': players[2].active = (players[2].active+1)%2;  // Activo/Desactivo Sample 2
                    if(players[2].active) players[2].index = 0;
        	    break;
        case '4': players[3].active = (players[3].active+1)%2;  // Activo/Desactivo Sample 2
                    if(players[3].active) players[3].index = 0;
                break;
        case '5': players[4].active = (players[4].active+1)%2;  // Activo/Desactivo Sample 2
                     if(players[4].active) players[4].index = 0;
                 break;
        case '6': players[5].active = (players[5].active+1)%2;  // Activo/Desactivo Sample 2
                     if(players[5].active) players[5].index = 0;
                 break;
        case 'p':
        	pianoMode =! pianoMode;
            if(pianoMode) {
            	LPC_GPIO0->FIOCLR |=(PIN_22);
            }
            else{ LPC_GPIO0->FIOSET|=(PIN_22);}
	    break;
        }
    }
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

    players[2].data = sampleS2;
    players[2].length = SAMPLE_S2_LEN;
    players[2].index = 0;
    players[2].active = 0;

    players[3].data = sampleS3;
    players[3].length = SAMPLE_S3_LEN;
    players[3].index = 0;
    players[3].active = 0;

    players[4].data = sampleS4;
    players[4].length = SAMPLE_S4_LEN;
    players[4].index = 0;
    players[4].active = 0;

    players[5].data = sampleS6;
    players[5].length = SAMPLE_S6_LEN;
    players[5].index = 0;
    players[5].active = 0;
    }

void sendWelcomeMessage(void) {
    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"\r\n===============================\r\n", 35, BLOCKING);

    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"     CONSOLA DE MEZCLA 3000     \r\n", 37, BLOCKING);

    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"===============================\r\n", 35, BLOCKING);

    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"[1]  SAMPLE_BAT\r\n", 18, BLOCKING);

    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"[2]  SAMPLE_PUM\r\n", 18, BLOCKING);

    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"[3]  SAMPLE_BOING\r\n", 20, BLOCKING);

    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"[4]  SAMPLE_INTERF\r\n", 20, BLOCKING);

    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"[5]  SAMPLE_VIRUS\r\n", 20, BLOCKING);
    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"[6]  SENOIDAL_PRUEBA\r\n", 20, BLOCKING);
    UART_Send((LPC_UART_TypeDef *)LPC_UART0,
              (uint8_t *)"\r\nSeleccione un sample con 1, 2 o 3...\r\n", 41, BLOCKING);
}

