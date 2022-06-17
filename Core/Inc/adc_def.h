#ifndef INC_ADC_DEF_H_
#define INC_ADC_DEF_H_

/* Defines BEGIN */
#define ADCMAX 4095.0
#define SHUNT 0.01
#define ADCREF 3.3
#define ADCVREFINT 1.2

/* Internal temperature sensor BEGIN */
#define ADC_TMP_V25 0.76 /* V */
#define ADC_TMP_AVGSLOPE 2.5 /* mV/°C */
/* Internal temperature sensor END */


#define ADC_OUT_DIV 45.64 //64.83 //64.83 /* R1=300k, R2=4.7k */
#define ADC_SHUNT 0.1 /* 1/10Ohm, where Rshunt=10 Ohm */

#define ADC_RSHUNT 10
#define ADC_RCURLIM 62


/* DEFINES END */

#endif /* INC_ADC_DEF_H_ */
