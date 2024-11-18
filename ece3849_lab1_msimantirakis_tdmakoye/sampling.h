#ifndef SAMPLING_H_
#define SAMPLING_H_
#define ADC_OFFSET       128
#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define PIXELS_PER_DIV 21
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer

// initalize
void ADC_Init(void);
void ButtonISR(void);
#endif //Ends the #ifndef SAMPLING_H_
