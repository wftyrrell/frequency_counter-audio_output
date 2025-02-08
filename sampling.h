 /*
 * sampling.h
 *
 *  Created on: Nov 13, 2022
 *      Author: wftyr
 */


#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>


#define VIN_RANGE 3.3                                    // total V_in range
#define PIXELS_PER_DIV 20                                // LCD pixels per voltage division
#define ADC_BITS 12                                      // number of bits in ADC
#define ADC_BUFFER_SIZE 2048
#define ADC_OFFSET 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))

void adcInit(void);
int RisingTrigger(void);
int FallingTrigger(void);
void ADC_ISR(void);

int32_t getADCBufferIndex(void);

#endif /* SAMPLING_H_ */
