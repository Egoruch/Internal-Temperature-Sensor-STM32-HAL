#ifndef INC_TMPSENSOR_H_
#define INC_TMPSENSOR_H_

/* More information:
 * https://cxemka.com
 * https://github.com/Egoruch?tab=repositories
 */

#include "main.h"

/* Configurations BEGIN */
#define TMPSENSOR_USE_INTREF 1 /* 1 - Use Internal Reference Voltage; 0 - Not use; */
/* Configurations END */

/* Constant values BEGIN */
#define TMPSENSOR_AVGSLOPE 2.5 /* mV/°C */
#define TMPSENSOR_V25  0.76 /* V (at 25 °C)  */

#define TMPSENSOR_ADCMAX 4095.0 /* 12-bit ADC maximum value (12^2)-1)  */
#define TMPSENSOR_ADCREFVOL  3.3 /* Typical reference voltage, V  */
#define TMPSENSOR_ADCVREFINT  1.2 /* Internal reference voltage, V  */
/* Constant values END */


/* PFP BEGIN */
double TMPSENSOR_getTemperature(uint16_t adc_sensor, uint16_t adc_intref);
/* PFP END */

#endif /* INC_TMPSENSOR_H_ */
