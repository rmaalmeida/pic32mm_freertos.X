/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC24 / dsPIC33 / PIC32MM MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - pic24-dspic-pic32mm : v1.35
        Device            :  PIC32MM0064GPL036
    The generated drivers are tested against the following:
        Compiler          :  XC32 1.42
        MPLAB             :  MPLAB X 3.60
 */

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
 */

#include "mcc_generated_files/mcc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/*
                         Main application
 */

int16_t pid_sw_fixed(int ad);
int16_t sp;

static void prvBlinkyTimerCallback(TimerHandle_t xTimer) {
    IO_RA0_Toggle();
}

static void task1(void *pvParameters) {
    uint16_t ad;
    uint16_t res;
    int t;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    ADC1_ChannelSelect(ADC1_CHANNEL_AN10);
    SCCP2_COMPARE_Start();
    for (;;) {
        ADC1_Start();
        for (t = 0; t < 1000; t++);
        ADC1_Stop();
        while (!ADC1_IsConversionComplete());
        ad = ADC1_ConversionResultGet();
        res = pid_sw_fixed(ad);
        SCCP2_COMPARE_DualCompareValueSet(0, sp);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void main(void) {

    // initialize the device
    SYSTEM_Initialize();


    // When using interrupts, you need to set the Global Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalDisable();

    xTaskCreate(task1, /* The function that implements the task. */
            "Rx", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
            configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
            NULL, /* The parameter passed to the task - just to check the functionality. */
            tskIDLE_PRIORITY + 1, /* The priority assigned to the task. */
            NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

void vApplicationMallocFailedHook(void) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void) pcTaskName;
    (void) pxTask;

    /* Run time task stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is 
    called if a task stack overflow is detected.  Note the system/interrupt
    stack is not checked. */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/*-----------------------------------------------------------*/


void vAssertCalled(const char * pcFile, unsigned long ulLine) {
    volatile unsigned long ul = 0;

    (void) pcFile;
    (void) ulLine;

    __asm volatile( "di");
    {
        /* Set ul to a non-zero value using the debugger to step out of this
        function. */
        while (ul == 0) {
            portNOP();
        }
    }
    __asm volatile( "ei");
}


/**
 End of File
 */

float f_e0 = 0, f_e1 = 0, f_e2 = 0;
float f_y0 = 0, f_y1 = 0;
const float f_kp = 1, f_ki = 50, f_kd = 0;
const float f_T = 0.005;

#define SHIFT (256)
int16_t i_e0 = 0, i_e1 = 0, i_e2 = 0;
int16_t i_y0 = 0, i_y1 = 0;
int16_t sp;


#define MAX_SAT 1023
#define MIN_SAT 0

int16_t pid_sw_fixed(int ad) {
    const int16_t i_k1 = (f_kp + f_ki * f_T + f_kd / f_T) * SHIFT;
    const int16_t i_k2 = -((f_kp + 2 * f_kd / f_T) * SHIFT);
    const int16_t i_k3 = (f_kd / f_T) * SHIFT;

    // Update variables
    i_y1 = i_y0;
    i_e2 = i_e1;
    i_e1 = i_e0;

    //ad = READ_AD())
    i_e0 = (sp - ad);

    // Processing
    //the multiplication is for a number range in 4.2(6 bits) (gains from zero up to +15.75) 
    // times 0.3ff (10 bits)
    //    y0 = y1 + (i_kp * (e0 - e2)) +
    //            (i_ki * (e0) * i_T) +
    //            (i_kd * (e0 - (2 * e1) + e2) / i_T);
    //stated in terms of errors instead of coefficients
    i_y0 = (((int32_t) i_k1 * i_e0) + ((int32_t) i_k2 * i_e1) + ((int32_t) i_k3 * i_e2));
    i_y0 = i_y0 >> 8;
    i_y0 += i_y1;

    // Saturation
    if (i_y0 > MAX_SAT) {
        i_y0 = MAX_SAT;
    } else if (i_y0 < MIN_SAT) {
        i_y0 = MIN_SAT;
    }

    return i_y0;
}