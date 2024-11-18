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


uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second
bool flag= false;
void ButtonISR(void);

//ISR(100 ms) conversion to time mm:ss:ff
void new_time(int time, int *minutes, int *seconds, int *fractions){
    *minutes = time / (60 * 100);
    *seconds = (time / 100) % 60;
    *fractions = (time % 100);
    // " * " is a pointer
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

int main(void)
{
    IntMasterDisable();

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
    IntMasterEnable();

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

int i, y,y2;
bool start_grid = true;

uint16_t sample[LCD_HORIZONTAL_MAX];
    while (true) {
        //Run only ones
        if(start_grid){
        //Starting Grid
        GrContextForegroundSet(&sContext, ClrBlue);
        for(i = 1; i < 128; i+=PIXELS_PER_DIV){
            GrLineDrawH(&sContext, /*x*/ 0, /*x*/ 128, /*y*/ i);
            GrLineDrawV(&sContext, /*x*/ i, /*y*/ 0, /*y*/ 128);
        }
        //Starting Center Lines
        GrContextForegroundSet(&sContext, ClrCrimson);
        GrLineDrawH(&sContext, /*x*/ 0, /*x*/ 128, /*y*/ 64);
        GrLineDrawV(&sContext, /*x*/ 64, /*y*/ 0, /*y*/ 128);

        start_grid = false;
        }
        // Draw wave
        if(flag){

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        //Draw Grid Again
        GrContextForegroundSet(&sContext, ClrBlue);
        for(i = 1; i < 128; i+=PIXELS_PER_DIV){
           GrLineDrawH(&sContext, /*x*/ 0, /*x*/ 128, /*y*/ i);
           GrLineDrawV(&sContext, /*x*/ i, /*y*/ 0, /*y*/ 128);
       }

        //Center Lines
        GrContextForegroundSet(&sContext, ClrCrimson);
        GrLineDrawH(&sContext, /*x*/ 0, /*x*/ 128, /*y*/ 64);
        GrLineDrawV(&sContext, /*x*/ 64, /*y*/ 0, /*y*/ 128);

        GrContextForegroundSet(&sContext, ClrYellow);
        for(i = 0;i<LCD_HORIZONTAL_MAX-1;i++){
                  sample[i] = gADCBuffer[ADC_BUFFER_WRAP(LCD_HORIZONTAL_MAX/2+i)];
                  y = LCD_VERTICAL_MAX/2 - (int)roundf(((int)sample[i]-ADC_OFFSET));
              }
              for(i = 0;i<LCD_HORIZONTAL_MAX-1;i++){
                  y = LCD_VERTICAL_MAX/2 - (int)roundf(((int)sample[i]-ADC_OFFSET)/36)+55;
                  GrLineDraw(&sContext, i, y2, i + 1, y);
                  y2=y;
              }

        flag = false;
        }
        GrFlush(&sContext); // flush the frame buffer to the LCD
    //End While Loop!!!

    }
}
