#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>              // rand()
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de Timers
#include "driverlib/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h "
#include "configADC.h"

#define PERIOD_PWM 12500    // TODO: Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
                                //625KHz/50 = 12500 para tener 50Hz = 20ms periodo
#define COUNT_1MS 625   // TODO: Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
                            // (12500ticks/20ms)*1ms = 625ticks reloj
#define STOPCOUNT 950  // TODO: Ciclos para amplitud de pulso de parada (1.52ms)
                            // (12500ticks/20ms)*1.52ms = 950ticks reloj
#define COUNT_2MS 1250   // TODO: Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
                            // (12500ticks/20ms)*2ms = 1250ticks reloj
#define NUM_STEPS 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms

#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud tras pulsacion

#define MITAD_1MS ((abs(COUNT_1MS - STOPCOUNT) / 2) + COUNT_1MS)

#define MITAD_2MS (COUNT_2MS - (abs(COUNT_2MS - STOPCOUNT) / 2))

#define LENTO_2MS ( (COUNT_2MS - ((abs(COUNT_2MS - STOPCOUNT) / 2))) - 60)

#define LENTO_1MS ( ((abs(COUNT_1MS - STOPCOUNT) / 2) + COUNT_1MS) + 100)

#define MASTER_TASK_PRIORITY tskIDLE_PRIORITY

#define valor_4cm 1100 //valor que leen los sharp de los lados cuando hay algo a 4cm

// Variables globales
#define MOVIENDO 0
#define LIMITE 1
int estado = MOVIENDO;
unsigned cont_buscar = 0;
int done = 0;

//MANEJADORES
TimerHandle_t TimerAtras;
TimerHandle_t TimerGiro;
TimerHandle_t TimerSharp;
TimerHandle_t TimerInit;

//LUT PARA LOS SHARP
unsigned short LUT_ADC[15] = {499, 552, 611, 741, 819, 924, 1060, 1247, 1433, 1600, 1777, 2028, 2329, 2599, 3092};
unsigned short centimetros[15]  = {30, 28, 24, 20, 18, 16, 14, 12, 10,9, 8, 7, 6, 5, 4};

void AtrasCallback(TimerHandle_t TimerAtras);
void GiroCallback(TimerHandle_t TimerGiro);
void SharpCallback(TimerHandle_t TimerSharp);
void InitCallback(TimerHandle_t TimerInit);
unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax);

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//#if 0
void vApplicationIdleHook( void )
{
        SysCtlSleep();
        //SysCtlDeepSleep();
}
//#endif


//*****************************************************************************
//
// This hook is called by FreeRTOS when a timer SW is used
//
//*****************************************************************************
static portTASK_FUNCTION(SharpTask,pvParameters)
{
    MuestrasADC muestras;
    LadosADC lados; //supongo chan1 = izq, chan2 = der (visto de frente)
    unsigned short indice;
    //int done = 0;
    while(1)
    {
        if(estado == MOVIENDO)
        {
            configADC_LeeADC0(&muestras); //Leemos lo que tenemosd delante
            configADC_LeeADC1(&lados);    //Leemos lo que tenemosd a los lados
            indice = binary_lookup(&LUT_ADC, muestras.chan1, 0, 14);
            if(centimetros[indice] <= 14)
            {
                if((lados.chan1 >= valor_4cm) && (lados.chan2 >= valor_4cm)) //lo tengo justo delante
                {
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS); // A machete
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS);
                }
                else if((lados.chan1 >= valor_4cm) && (lados.chan2 <= valor_4cm)) //lo tengo a la izq
                {
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, LENTO_1MS); // VOY GIRANDO A LA IZQUIERDA
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS);
                }
                else if((lados.chan2 >= valor_4cm) && (lados.chan1 <= valor_4cm)) //lo tengo a la der
                {
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS); // VOY GIRANDO A LA DERECHA
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, LENTO_2MS);
                }
                else // lo tengo lejos, intento acercarme
                {
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS); // A machete
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS);
                }
                xTimerStop(TimerSharp, portMAX_DELAY );
                done = 0;
                cont_buscar = 0;
            }
            else
            {
                if(done == 0)
                {
                    xTimerStart(TimerSharp, portMAX_DELAY );
                    done = 1;
                }
            }
        }
        IntEnable(INT_ADC0SS3);
        IntEnable(INT_ADC1SS2); //cambio
    }
}

//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int main(void)
{
    // Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable
    // Reloj del sistema a 40MHz (PLL-200MHz/5=40MHz)
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);    // Establece reloj del sistema (40MHz/64=625KHz)
    SysCtlPeripheralClockGating(true); //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
                                                                                                               // deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);

    //Config PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Habilita el puerto F
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM (PF2 y PF3 funcionan con el modulo PWM1, mirar en pi_map.h)
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);              // del módulo PWM1 para PF2
    GPIOPinConfigure(GPIO_PF3_M1PWM7);              // del módulo PWM1 para PF3
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3); // PF2 y  PF3 como salida PWM

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM de base 0 y generador 3 para los PF2 y PF3 contara hacia abajo
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);    // Carga la cuenta que establece la frecuencia de la señal PWM


    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, MITAD_1MS); //Inicializamos el ancho de pulso avanzando
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, MITAD_2MS); //Inicializamos el ancho de pulso avanzando

    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la señal RUEDA IZQUIERDA
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la señal RUEDA DERECHA

    //PWM bandera
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Habilita el puerto E
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //Habilita modulo PWM (PE4 funcionan con el modulo PWM0, mirar en pi_map.h)
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4); // PE4 como salida PWM
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);   // Módulo PWM de base 0 y generador 3 para los PF2 y PF3 contara hacia abajo
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PERIOD_PWM);    // Carga la cuenta que establece la frecuencia de la señal PWM
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 450); //Inicializamos el ancho de pulso para 0 grados
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true); // Habilita la salida de la señal RUEDA DERECHA
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    //GPIO A:

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Habilita el puerto A
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);

    //ENCODER

    ROM_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN); //A5 -> SUELO ATRAS, A6 -> SUELO DELANTE
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_HIGH_LEVEL); //GPIO_HIGH_LEVEL -> Queremos detectar Blanco
    MAP_IntPrioritySet(INT_GPIOA, configMAX_SYSCALL_INTERRUPT_PRIORITY); //configMAX_SYSCALL_INTERRUPT_PRIORITY

    //SW TIMERS
    TimerAtras = xTimerCreate("TimerAtras",  configTICK_RATE_HZ/2, pdFALSE, NULL, AtrasCallback); // pdFALSE para que no sea ciclico
    TimerGiro = xTimerCreate("TimerGiro",  configTICK_RATE_HZ/4, pdFALSE, NULL, GiroCallback); // pdFALSE para que no sea ciclico
    TimerSharp = xTimerCreate("TimerSharp",  0.5*configTICK_RATE_HZ, pdFALSE, NULL, SharpCallback); // pdFALSE para que no sea ciclico
    TimerInit = xTimerCreate("TimerInit",  5*configTICK_RATE_HZ, pdFALSE, NULL, InitCallback); // pdFALSE para que no sea ciclico


    //CONFIGURACION ADC
    configADC_IniciaADC();

    uint32_t ui32Period;
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ui32Period = SysCtlClockGet()/40; //periodo de 25ms
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);

    //CONFIGURACION BOTON 2
    ButtonsInit();
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE); // Configura pulsadores placa TIVA (int. por flanco de bajada)
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    IntEnable(INT_GPIOF);

    //Tarea

    if((xTaskCreate(SharpTask, (portCHAR *)"SharpTask", 512, NULL,MASTER_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }

    vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asa que no hace falta habilitarlas

    while(1)
    {
    }
}
void InitCallback(TimerHandle_t TimerInit)
{
    TimerEnable(TIMER2_BASE, TIMER_A); //si no va el timer no hay interrupciones del ADC
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 900); //Inicializamos el ancho de pulso para 90 grados
    IntEnable(INT_GPIOA); // interrupciones sensor de suelo
    xTimerStop( TimerInit, portMAX_DELAY );
}
void AtrasCallback (TimerHandle_t TimerAtras)
{
    //CUANDO SALTE ESTE TIMER EMPIEZO A ROTAR
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, MITAD_2MS); //Rota DERECHA
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, MITAD_2MS); //Rota IZQUIERDA
    xTimerStart(TimerGiro, portMAX_DELAY );
    xTimerStop( TimerAtras, portMAX_DELAY );
}

void GiroCallback (TimerHandle_t TimerGiro)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, MITAD_1MS); //AVANZA
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, MITAD_2MS);
    estado = MOVIENDO;
    IntEnable(INT_ADC0SS3);
    IntEnable(INT_ADC1SS2);
    xTimerStart(TimerSharp, portMAX_DELAY );
    xTimerStop(TimerGiro, portMAX_DELAY);
}
void SharpCallback (TimerHandle_t TimerSharp)
{
    if(estado == MOVIENDO)
    {
        if(cont_buscar < 2) //un switch case seria mas elegante...
        {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, MITAD_1MS); //AVANZA
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, MITAD_2MS);
            cont_buscar++;
        }
        else if(cont_buscar == 2)
        {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, MITAD_2MS); //Rota DERECHA
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, MITAD_2MS);
            cont_buscar++;
        }
        else if(cont_buscar >= 3 && cont_buscar < 5)
        {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, MITAD_1MS); //Rota IZQUIERDA
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, MITAD_1MS);
            cont_buscar++;
        }
        else
        {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, MITAD_1MS); //AVANZA
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, MITAD_2MS);
            cont_buscar = 0;
        }

    }
    done = 0;
    xTimerStop(TimerSharp, portMAX_DELAY);
}

unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax)
{
  unsigned int imid;

  while (imin < imax)
    {
      imid= (imin+imax)>>1;

      if (A[imid] < key)
        imin = imid + 1;
      else
        imax = imid;
    }
    return imax;    //Al final imax=imin y en dicha posicion hay un numero mayor o igual que el buscado
}

void GPIOFIntHandler (void) //INTERRUPCIONES BOTON 2
{
    BaseType_t higherPriorityTaskWoken=pdFALSE; //Hay que inicializarlo a False!!
    uint32_t IntStatus;
    IntStatus = GPIOIntStatus(GPIO_PORTF_BASE,true); //status devuelve el estado del puerto F
    if (IntStatus&GPIO_PIN_0)
    {
        xTimerStartFromISR(TimerInit, &higherPriorityTaskWoken ); // llamo al timer
        IntDisable(INT_GPIOF); // No quiero que vuelva  a saltar

    }

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken)
}

void GPIOAIntHandler (void) //INTERRUPCIONES SENSOR SUELO
{
    BaseType_t higherPriorityTaskWoken=pdFALSE; //Hay que inicializarlo a False!!
    uint32_t IntStatus;

    IntStatus=GPIOIntStatus(GPIO_PORTA_BASE,true); //status devuelve el estado del puerto A


    if(IntStatus&GPIO_PIN_6) // SUELO DELANTE
    {
        //DETECTO BLANCO DELANTE
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_2MS); // EMPIEZO A RETROCEDER
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_1MS);
        xTimerStartFromISR(TimerAtras, &higherPriorityTaskWoken ); // llamo al timer
        ROM_IntDisable(INT_TIMER1A);
        estado = LIMITE;
    }
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5); // LIMPIO EL FLAG
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6); // LIMPIO EL FLAG

    portEND_SWITCHING_ISR(higherPriorityTaskWoken)
}
