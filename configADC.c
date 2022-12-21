// Author: manumnadal & Oscar PF

#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define INT_LOW_PRIORITY tskIDLE_PRIORITY+1

static QueueHandle_t cola_adc0;
static QueueHandle_t cola_adc1;


void configADC_IniciaADC(void)
{
                //Habilitamos ADC0 para sensor Derecho: Pin E3/AIN0 -> ADC0
                SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
                SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

                //Habilitamos ADC1 para sensor Izquierdo Pin E2/AIN1 & E1/AIN3-> ADC1
                SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
                SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC1);

                //HABILITAMOS EL GPIOE
                SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
                SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
                GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);

                //CONFIGURAR SECUENCIADOR 3
                ADCSequenceDisable(ADC0_BASE,3); //Cogemos Secuenciador 3 porque solo vamos a tomar una muestra cada vez
                ADCSequenceDisable(ADC1_BASE,2);

                ADCHardwareOversampleConfigure(ADC0_BASE,64); //Sobremuestreo para reducir el ruido
                ADCHardwareOversampleConfigure(ADC1_BASE,64);

                ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1); //Configuramos la velocidad de conversion al maximo (1MS/s)
                ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_RATE_FULL, 1); //Configuramos la velocidad de conversion al maximo (1MS/s)

                ADCSequenceConfigure(ADC0_BASE,3,ADC_TRIGGER_TIMER,0);  //Disparo por timer
                ADCSequenceConfigure(ADC1_BASE,2,ADC_TRIGGER_TIMER,0);  //Disparo por timer

                ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

                ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH1);
                ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

                ADCSequenceEnable(ADC0_BASE,3); //ACTIVO LA SECUENCIA
                ADCSequenceEnable(ADC1_BASE,2); //ACTIVO LA SECUENCIA

                //Habilita las interrupciones ADC0
                ADCIntClear(ADC0_BASE,3);
                ADCIntEnable(ADC0_BASE,3);
                IntPrioritySet(INT_ADC0SS3,configMAX_SYSCALL_INTERRUPT_PRIORITY);
                IntEnable(INT_ADC0SS3);

                //Habilita las interrupciones ADC1
                ADCIntClear(ADC1_BASE,2);
                ADCIntEnable(ADC1_BASE,2);
                IntPrioritySet(INT_ADC1SS2,configMAX_SYSCALL_INTERRUPT_PRIORITY);
                IntEnable(INT_ADC1SS2);

                //Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
                cola_adc0=xQueueCreate(8,sizeof(MuestrasADC));
                if (cola_adc0==NULL)
                {
                    while(1);
                }
                cola_adc1=xQueueCreate(8,sizeof(LadosADC));
                if (cola_adc1==NULL)
                {
                    while(1);
                }
}


void configADC_LeeADC0(MuestrasADC *datos)
{
    xQueueReceive(cola_adc0,datos,portMAX_DELAY);
}
void configADC_LeeADC1(LadosADC *datos)
{
    xQueueReceive(cola_adc1,datos,portMAX_DELAY);
}
void configADC0_ISR(void)
{
    portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

    MuestrasLeidasADC leidas;
    MuestrasADC finales;
    ADCIntClear(ADC0_BASE,3);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ADCSequenceDisable(ADC0_BASE,3);
    ADCSequenceDataGet(ADC0_BASE,3,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS
    ADCSequenceEnable(ADC0_BASE,3);
    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)
    finales.chan1=leidas.chan1;
    //Guardamos en la cola
    xQueueSendFromISR(cola_adc0,&finales,&higherPriorityTaskWoken);
    IntDisable(INT_ADC0SS3);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
void configADC1_ISR(void)
{
    portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

    LadosLeidosADC leidas;
    LadosADC finales;
    ADCIntClear(ADC1_BASE,2);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ADCSequenceDisable(ADC1_BASE,2);
    ADCSequenceDataGet(ADC1_BASE,2,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS
    ADCSequenceEnable(ADC1_BASE,2);
    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)
    finales.chan1=leidas.chan0;
    finales.chan2=leidas.chan1;
    //Guardamos en la cola
    xQueueSendFromISR(cola_adc1,&finales,&higherPriorityTaskWoken);
    IntDisable(INT_ADC1SS2);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

