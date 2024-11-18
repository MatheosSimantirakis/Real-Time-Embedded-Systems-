/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-B06
 */

#include <xdc/std.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>
extern const ti_sysbios_family_arm_m3_Hwi_Handle m3Hwi0;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle Waveform;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle Processing;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle Display;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle waveCheck;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle processingCheck;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle displayCheck;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle Buttonclockfunction;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle Button;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle UserInput;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle button;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle buttonMailbox;

#include <ti/sysbios/knl/Mailbox.h>
extern const ti_sysbios_knl_Mailbox_Handle ButtonMailbox;

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

