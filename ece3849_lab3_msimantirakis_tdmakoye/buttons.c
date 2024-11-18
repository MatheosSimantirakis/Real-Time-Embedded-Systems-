/*
 * buttons.c
 *
 *  Created on: Aug 12, 2012, modified 9/8/2017
 *      Author: Gene Bogdanov
 *
 * ECE 3849 Lab button handling
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "buttons.h"
#include "sampling.h"

// public globals
volatile uint32_t gButtons = 0; // debounced button state, one per bit in the lowest bits
                                // button is pressed if its bit is 1, not pressed if 0
uint32_t gJoystick[2] = {0};    // joystick coordinates
uint32_t gADCSamplingRate;      // [Hz] actual ADC sampling rate

//Lab 2
//typedef char DataType;      // FIFO data type
volatile DataType fifo[FIFO_SIZE];  // FIFO storage array
volatile int fifo_head = 0; // index of the first item in the FIFO
volatile int fifo_tail = 0; // index one step past the last item

// imported globals
extern uint32_t gSystemClock;   // [Hz] system clock frequency
extern volatile uint32_t gTime; // time in hundredths of a second
extern bool flag;

//Lab2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile float timeScaling[] = {100e-3, 50e-3, 20e-3, 10e-3, 5e-3, 2e-3, 1e-3, 500e-6, 200e-6, 100e-6, 50e-6, 20e-6};


// initialize all button and joystick handling hardware
void ButtonInit(void)
{
    // initialize a general purpose timer for periodic interrupts
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, roundf((float)gSystemClock / BUTTON_SCAN_RATE) - 1);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_BOTH);

    // initialize interrupt controller to respond to timer interrupts
    IntPrioritySet(INT_TIMER0A, BUTTON_INT_PRIORITY);
    IntEnable(INT_TIMER0A);

    // GPIO PJ0 and PJ1 = EK-TM4C1294XL buttons 1 and 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //JOYSTICK_SETUP---------------------------------------------------------------------------

    // analog input AIN13, at GPIO PD2 = BoosterPack Joystick HOR(X)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    // analog input AIN17, at GPIO PK1 = BoosterPack Joystick VER(Y)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    // digital input for Joystick Z = Select
    //initialize Port D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //initialize Pin_4 of Port_D as Input
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_4);
    //Configure PD_4 current and button type
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //-----------------------------------------------------------------------------------------


    //BUTTON_SETUP----------------------------------------------------------------------------

    //Button 1 BOOSTER: GPIO_PH1
    //initialize Port H
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    //initialize Pin_1 of Port_H as Input
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);
    //Configure PH_1 current and button type
    GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Button 2 BOOSTER: GPIO_PK6
    //initialize Port K
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    //initialize Pin_6 of Port_K as Input
    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);
    //Configure PK_6 current and button type
    GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


    //-----------------------------------------------------------------------------------------


    // initialize ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor); // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor); // only ADC0 has PLL clock divisor control

    // initialize ADC sampling sequence
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);                             // Joystick HOR(X)
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);  // Joystick VER(Y)
    ADCSequenceEnable(ADC0_BASE, 0);

}

// update the debounced button state gButtons
void ButtonDebounce(uint32_t buttons)
{
	int32_t i, mask;
	static int32_t state[BUTTON_COUNT]; // button state: 0 = released
									    // BUTTON_PRESSED_STATE = pressed
									    // in between = previous state
	for (i = 0; i < BUTTON_COUNT; i++) {
		mask = 1 << i;
		if (buttons & mask) {
			state[i] += BUTTON_STATE_INCREMENT;
			if (state[i] >= BUTTON_PRESSED_STATE) {
				state[i] = BUTTON_PRESSED_STATE;
				gButtons |= mask; // update debounced button state
			}
		}
		else {
			state[i] -= BUTTON_STATE_DECREMENT;
			if (state[i] <= 0) {
				state[i] = 0;
				gButtons &= ~mask;
			}
		}
	}
}

// sample joystick and convert to button presses
void ButtonReadJoystick(void)
{
    ADCProcessorTrigger(ADC0_BASE, 0);          // trigger the ADC sample sequence for Joystick X and Y
    while(!ADCIntStatus(ADC0_BASE, 0, false));  // wait until the sample sequence has completed
    ADCSequenceDataGet(ADC0_BASE, 0, gJoystick);// retrieve joystick data
    ADCIntClear(ADC0_BASE, 0);                  // clear ADC sequence interrupt flag

    // process joystick movements as button presses using hysteresis
    if (gJoystick[0] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 5; // joystick right in position 5
    if (gJoystick[0] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 5);

    if (gJoystick[0] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 6; // joystick left in position 6
    if (gJoystick[0] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 6);

    if (gJoystick[1] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 7; // joystick up in position 7
    if (gJoystick[1] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 7);

    if (gJoystick[1] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 8; // joystick down in position 8
    if (gJoystick[1] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 8);
}

// autorepeat button presses if a button is held long enough
uint32_t ButtonAutoRepeat(void)
{
    static int count[BUTTON_AND_JOYSTICK_COUNT] = {0}; // autorepeat counts
    int i;
    uint32_t mask;
    uint32_t presses = 0;
    for (i = 0; i < BUTTON_AND_JOYSTICK_COUNT; i++) {
        mask = 1 << i;
        if (gButtons & mask)
            count[i]++;     // increment count if button is held
        else
            count[i] = 0;   // reset count if button is let go
        if (count[i] >= BUTTON_AUTOREPEAT_INITIAL &&
                (count[i] - BUTTON_AUTOREPEAT_INITIAL) % BUTTON_AUTOREPEAT_NEXT == 0)
            presses |= mask;    // register a button press due to auto-repeat
    }
    return presses;
}

//From Lectures
// put data into the FIFO, skip if full
// returns 1 on success, 0 if FIFO was full
int fifo_put(DataType data)
{
    int new_tail = fifo_tail + 1;
    if (new_tail >= FIFO_SIZE) new_tail = 0; // wrap around
    if (fifo_head != new_tail) {    // if the FIFO is not full
        fifo[fifo_tail] = data;     // store data into the FIFO
        fifo_tail = new_tail;       // advance FIFO tail index
        return 1;                   // success
    }
    return 0;   // full
}



//From Lectures
// get data from the FIFO
// returns 1 on success, 0 if FIFO was empty
int fifo_get(DataType *data)
{
    if (fifo_head != fifo_tail) {   // if the FIFO is not empty
        *data = fifo[fifo_head];    // read data from the FIFO
        fifo_head++;                // advance FIFO head index
        if (fifo_head >= FIFO_SIZE) fifo_head = 0; // wrap around
        return 1;                   // success
    }
    return 0;   // empty
}


// ISR for scanning and debouncing buttons
void ButtonISR(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

    // read hardware button state
    uint32_t gpio_buttons =
            ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0) | // EK-TM4C1294XL buttons in positions 0 and 1
            (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1 |
            (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3 |
            ~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4);

    uint32_t old_buttons = gButtons;              // save previous button state
    ButtonDebounce(gpio_buttons);                 // Run the button debouncer. The result is in gButtons.
    ButtonReadJoystick();                         // Convert joystick state to button presses. The result is in gButtons.
    uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
    presses |= ButtonAutoRepeat();                // autorepeat presses if a button is held long enough

   // Old Button Code Lab0/1
//    static bool tic = false;
//    static bool running = true;
//    if (presses & 1) { // EK-TM4C1294XL button 1 pressed
//        running = !running;
//    }
//
//    if (presses & 2) { //Re_Starts Time when EK-TM4C1294XL button 2 pressed
//        gTime = 0;
//        flag= true;
//    }
//
//    if (running) {
//        if (tic) gTime++; // increment time every other ISR call
//        tic = !tic;
//    }


    //Buttons for Scaling using fifo_put
//      if(presses & 1){ //Button 1 on BaseBoard
//          fifo_put('t');//Time Change Selection
//      }else if(presses & 2){//Button 2 on BaseBoard
//          fifo_put('s');//Slope trigger Change Selection
//      }else if(presses & 4){//Button c on BoosterBoard
//          fifo_put('c');//Change Selected Setting (Time Scale or Slope Trigger)
//      }
}
uint32_t getButtonPresses(void){
    // read hardware button state
      uint32_t gpio_buttons =
              ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0  ); // EK-TM4C1294XL buttons in positions 0 and 1
      gpio_buttons = gpio_buttons|((~GPIOPinRead(GPIO_PORTH_BASE, 0xff)&GPIO_PIN_1)<<1);
      gpio_buttons = gpio_buttons|((~GPIOPinRead(GPIO_PORTD_BASE, 0xff)&GPIO_PIN_4));
      gpio_buttons = gpio_buttons|((~GPIOPinRead(GPIO_PORTK_BASE, 0xff)&GPIO_PIN_6)>>3);
      uint32_t old_buttons = gButtons;    // save previous button state
      ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
      ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
      uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
      presses |= ButtonAutoRepeat();      // autorepeat presses if a button is held long enough
      return presses;
}
//Lab2
//uint32_t triggerSearch(void){
//    int triggerIndex;
//    int offset = 2047; //half of max sample values 4095
//    int localBufferIndex = gADCBufferIndex;
//    int i;
//
//    for(i = 0; i >= (ADC_BUFFER_SIZE/2); i++){
//        triggerIndex = localBufferIndex - i;
//        //Finding Zero Points within gADCBuffer Data
//        if((gADCBuffer[triggerIndex] >= offset) && (gADCBuffer[triggerIndex + i] <= offset)){
//            return triggerIndex;
//           }
//    }
//    return 0;
//}

