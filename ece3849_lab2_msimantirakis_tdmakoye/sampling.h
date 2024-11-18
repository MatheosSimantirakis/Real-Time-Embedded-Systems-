#ifndef SAMPLING_H_
#define SAMPLING_H_
#define ADC_OFFSET       2048
#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define VIN_RANGE 3.3       // global voltage input range
#define PIXELS_PER_DIV 20   // determines the pixel per division on the oscilloscope
#define ADC_BITS 12         // the ADC has 12 bits


// initalize
void ADC_Init(void);
void ButtonISR(void);
int RisingTrigger(void);
int FallingTrigger(void);
#endif //Ends the #ifndef SAMPLING_H_
