/*
 * config_adc.h
 *
 *  Created on: 29 oct. 2022
 *      Author: manumnadal & Oscar PF
 */

#ifndef CONFIG_ADC_H_
#define CONFIG_ADC_H_


typedef struct
{
    uint16_t chan1;
} MuestrasADC;

typedef struct
{
    uint32_t chan1;
} MuestrasLeidasADC;

typedef struct
{
    uint16_t chan1;
    uint16_t chan2;
} LadosADC;

typedef struct
{
    uint32_t chan0;
    uint32_t chan1;
} LadosLeidosADC;

void configADC0_ISR(void);
void configADC1_ISR(void);
void configADC_LeeADC1(LadosADC *datos);
void configADC_LeeADC0(MuestrasADC *datos);
void configADC_IniciaADC(void);


#endif /* CONFIG_ADC_H_ */
