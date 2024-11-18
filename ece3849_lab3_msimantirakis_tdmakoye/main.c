/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <buttons.h>

#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"
#include "driverlib/timer.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

/*
 *  ======== main ========
 */
uint16_t sample[LCD_HORIZONTAL_MAX];
float fScale;
char str[50];
int trig=0;
int trigger;
//tContext in Globle Var.
tContext sContext;


//Lab2
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", "1 V", "2 V"}; //Array of Voltage Scaling [str]
const char * const gTimeScaleStr[] = {"100 ms", "50 ms", "20 ms", "10 ms", "5 ms", "2 ms", "1 ms", "500 us", "200 us", "100 us", "50 us", "20 us"};//Array of time scaling types (time/div) [str]
const char * const gTriggerSlopeStr[] = {"Rising", "Falling"};//Array of Slope settings [str]

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;
int VoltsIndex=0;
char buttonPress;

int i, y,y2;
bool start_grid = true;
volatile float Volts;
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2};
bool isRisingTrigger = true;

//CPU
uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

int triggerSearch(int triggerValue){
    if (triggerValue==1){
        triggerValue = RisingTrigger();
    }else if(triggerValue ==2){
        triggerValue = FallingTrigger();
    }
    return triggerValue;
}

////Function: Drawing Both Grid and Center Lines
void grid_lines(){

    int i = 0;

    GrContextForegroundSet(&sContext, ClrBlue);
    for(i = 1; i < 128; i+=21){
        GrLineDrawH(&sContext, /*x*/ 0, /*x*/ 128, /*y*/ i);
        GrLineDrawV(&sContext, /*x*/ i, /*y*/ 0, /*y*/ 128);
    }
    //Starting Center Lines
    GrContextForegroundSet(&sContext, ClrCrimson);
    GrLineDrawH(&sContext, /*x*/ 0, /*x*/ 128, /*y*/ 64);
    GrLineDrawV(&sContext, /*x*/ 64, /*y*/ 0, /*y*/ 128);
}

int main(void)
{
    IntMasterDisable();
    // hardware initialization goes here
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    //    FPUEnable();
    //    FPULazyStackingEnable();
    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation
    ADC_Init();
    //Button Initialize
    ButtonInit();
    //
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock *0.01); // 1 sec interval
    count_unloaded = cpu_load_count();

    /* Start BIOS */
    BIOS_start();

    return (0);
}


void Waveform_task(UArg arg1, UArg arg2){
    IntMasterEnable();
    while(true){
        Semaphore_pend(waveCheck, BIOS_WAIT_FOREVER); //
        //Collects DATA
        //        int trigger = (isRisingTrigger)? RisingTrigger(): FallingTrigger();
        //        if (isRisingTrigger == true){
        //                trig = 0;
        //             }else{
        //                 trig = 1;
        //             }
        //grid_lines();
        trigger = (isRisingTrigger)? RisingTrigger(): FallingTrigger();
//        if (isRisingTrigger == true){
//            trig = 0;
//        }else{
//            trig = 1;
//        }
        for(i = 0;i<LCD_HORIZONTAL_MAX-1;i++){
            sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger+i)];
        }
        Semaphore_post(processingCheck);//
    }

}
void Buttonclock_func(UArg arg){
    Semaphore_post(button);
}

void button_task(UArg arg1, UArg arg2){
    while(true){
        Semaphore_pend(button, BIOS_WAIT_FOREVER);
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

        static bool tic = false;
        static bool running = true;
        if (presses & 1) { // EK-TM4C1294XL button 1 pressed
            running = !running;
            buttonPress = 't';
            fifo_put('t');
            Mailbox_post(ButtonMailbox, &buttonPress, BIOS_WAIT_FOREVER);
        }

        if (presses & 2) { //Re_Starts Time when EK-TM4C1294XL button 2 pressed
            //              gTime = 0;
            buttonPress = 's';
//            fifo_put('s');
            Mailbox_post(ButtonMailbox, &buttonPress, BIOS_WAIT_FOREVER);
            //             flag= true;
        }

        if (running) {
            //              if (tic) gTime++; // increment time every other ISR call
            tic = !tic;
        }
        //        uint32_t presses = getButtonPresses();

    }
}


void UserInput_task(UArg arg1, UArg arg2){
    while(true){
        Mailbox_pend(ButtonMailbox, &buttonPress, BIOS_WAIT_FOREVER);
        if (buttonPress == 't'){ // increment state
            ++VoltsIndex;
            Volts = fVoltsPerDiv[VoltsIndex];
            fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * Volts);
            if(VoltsIndex>4){
                VoltsIndex=0;
            }
        }
        else if (buttonPress == 's'){ //trigger
            isRisingTrigger = !isRisingTrigger;
            if (isRisingTrigger == true){
                trig = 0;
            }else{
                trig = 1;
            }
        }
        Semaphore_post(displayCheck);
    }

}
void Processing_task(UArg arg1, UArg arg2){
    while(true){
        Semaphore_pend(processingCheck, BIOS_WAIT_FOREVER); //
        Semaphore_post(displayCheck);
        volatile int b = 1;
        Semaphore_post(waveCheck);
    }

}

void Display_task(UArg arg1, UArg arg2){
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
    while(true){
        Semaphore_pend(displayCheck, BIOS_WAIT_FOREVER); //
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        grid_lines();

        //Prints 128 samples of said Data
        GrContextForegroundSet(&sContext, ClrWheat);
        for(i = 0;i<LCD_HORIZONTAL_MAX-1;i++){
            y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale*((int)sample[i]-ADC_OFFSET));
            GrLineDraw(&sContext, i, y2, i + 1, y);
            y2=y;
        }
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load

        GrContextForegroundSet(&sContext, ClrWhite); // white text
        snprintf(str, sizeof(str), gTriggerSlopeStr[trig]);
        GrStringDraw(&sContext, str, -1, 85, 0, false);
        // Display voltage scale
        snprintf(str, sizeof(str), gVoltageScaleStr[VoltsIndex]);
        GrStringDraw(&sContext, str, -1, 40, 0, false);

        // Display CPU load %
        snprintf(str, sizeof(str), "CPU load = %.1f%%", cpu_load*100);
        GrStringDraw(&sContext, str, -1, 0, 120, false);
        GrFlush(&sContext); // flush the frame buffer to the LCD: IMPORTANT
        Semaphore_post(waveCheck);
    }
}


