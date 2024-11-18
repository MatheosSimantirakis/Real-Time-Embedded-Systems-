/**
 * main.c
 *
 * ECE 3849 Lab 0 Starter Project
 * Gene Bogdanov    10/18/2017
 *
 * This version is using the new hardware for B2017: the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <buttons.h>

//Lab_1
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#include "sampling.h"
#include "driverlib/timer.h"


uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second
bool flag= false;
void ButtonISR(void);
float fScale;
char str[50];
int trig=0;
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
//ISR(100 ms) conversion to time mm:ss:ff
void new_time(int time, int *minutes, int *seconds, int *fractions){
    *minutes = time / (60 * 100);
    *seconds = (time / 100) % 60;
    *fractions = (time % 100);
    // " * " is a pointer
}

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

void Signal_Init(){
    //Lab1
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,
                    roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

//Function: Drawing Both Grid and Center Lines
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


    int triggerSearch(int triggerValue);
    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    //Lab1
    Signal_Init();
    ADC_Init();

    //Button Initialize
    ButtonInit();


    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock - 1); // 1 sec interval

    count_unloaded = cpu_load_count();

    IntMasterEnable();

    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    int i, y,y2;
    bool start_grid = true;
    float Volts;

    uint16_t sample[LCD_HORIZONTAL_MAX];

    // uint32_t vState = 3;
    float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2};
    char buttonPress;//Stores button presses

    bool isRisingTrigger = true;


    //   IntMasterEnable();
    while (true) {
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load

        //Run only ones
        if(start_grid){
            grid_lines();
            start_grid = false;
        }

        //Scaling, Trigger, & Changing
        if (fifo_get(&buttonPress)) {
            // read bpresses and change state based on bpresses buttons and whether it is being pressed currently
            if (buttonPress == 't'){ // increment state
                ++VoltsIndex;
                Volts = fVoltsPerDiv[VoltsIndex];
                if(VoltsIndex>4){
                    VoltsIndex=0;
                }
            }
            else if (buttonPress == 's'){ //trigger
                isRisingTrigger = !isRisingTrigger;
            }

        }

        //Draw Grid and Center Lines Again
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        grid_lines();

        int trigger = (isRisingTrigger)? RisingTrigger(): FallingTrigger();
        if (isRisingTrigger == true){
                trig = 0;
             }else{trig = 1;}
        //        int trigger = triggerSearch(risefalltrigger);
        GrContextForegroundSet(&sContext, ClrYellow);
        fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * Volts);
        //Collects DATA
        for(i = 0;i<LCD_HORIZONTAL_MAX-1;i++){
            sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger+i)];
            //              y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale*((int)sample[i]-ADC_OFFSET));
        }
        //Prints 128 samples of said Data
        for(i = 0;i<LCD_HORIZONTAL_MAX-1;i++){
            y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale*((int)sample[i]-ADC_OFFSET));
            GrLineDraw(&sContext, i, y2, i + 1, y);
            y2=y;
        }

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
    }//End While Loop!!!

}
