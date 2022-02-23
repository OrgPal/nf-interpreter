//
// Copyright (c) 2020 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

//#include <stdint.h>
#include <nanoCLR_Application.h>
#include <nanoHAL_v2.h>

// RTOS header files
#include <xdc/std.h>
#include <ioc.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

// board Header files
#include <ti_drivers_config.h>
// clang-format off
#include DeviceFamily_constructPath(inc/hw_prcm.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
// clang-format on

// need this until fix in TI SDK is implemented
extern void PIN_init_nano();

//////////////////////////////

// Stack size in bytes
#define THREADSTACKSIZE 2048

Task_Struct receiverTask;
Task_Struct clrTask;

CLR_SETTINGS clrSettings;

// this define has to match the one in cpu_gpio.cpp
#define GPIO_MAX_PINS 16

extern void ReceiverThread(UArg arg0, UArg arg1);
extern void CLRStartupThread(UArg arg0, UArg arg1);

/////////////////////////////////////////////////

Task_Struct testTask;

void TestTaskExec(UArg arg0, UArg arg1)
{
    (void)arg0;
    (void)arg1;

    // loop forever
    while (1)
    {
        // Allow other tasks a chance to run
        Task_sleep(100);
    }
}

/////////////////////////////////////////////////

int main(void)
{
    Task_Params taskParams;

    // get and store reset reason
    // must be called before PIN_init()
    WakeupReasonStore = SysCtrlResetSourceGet();

    // hack required to be able to config wakeup from deep sleep
    // bug introduced in SDK 5.30.01.01.
    PIN_init_nano();

    // Call board init functions
    // Board_init();

    ConfigUART();

    // ////////////////////////////////////////////////////////////////////
    // *** FOR DEBUG RUNS, PLEASE KEEP THIS CODE ***
    // // setup Test task
    // Task_Params_init(&taskParams);
    // taskParams.stackSize = THREADSTACKSIZE;
    // taskParams.priority = 4;
    // // create task
    // Task_construct(&testTask, TestTaskExec, &taskParams, Error_IGNORE);
    // ////////////////////////////////////////////////////////////////////

    // Map LNA enable pin RFC_GPO0 to DIO21
    IOCPortConfigureSet(IOID_21, IOC_PORT_RFC_GPO0, IOC_IOMODE_NORMAL);

    // Map PA enable pin RFC_GPO1 to DIO22
    IOCPortConfigureSet(IOID_22, IOC_PORT_RFC_GPO1, IOC_IOMODE_NORMAL);

    // setup Task thread
    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.priority = 4;

    // create Receiver
    Task_construct(&receiverTask, ReceiverThread, &taskParams, Error_IGNORE);

    // CLR settings to launch CLR thread
    (void)memset(&clrSettings, 0, sizeof(CLR_SETTINGS));

    clrSettings.MaxContextSwitches = 50;
    clrSettings.WaitForDebugger = false;
    clrSettings.EnterDebuggerLoopAfterExit = true;

    // setup CLR task
    taskParams.arg0 = (UArg)&clrSettings;
    taskParams.stackSize = 2 * THREADSTACKSIZE;
    taskParams.priority = 4;

    Task_construct(&clrTask, CLRStartupThread, &taskParams, Error_IGNORE);

    BIOS_start();

    return (0);
}

///////////////////////////////////////////////////////////////////////
// need this dummy implementation here (started with SDK 4.20.01.04) //
///////////////////////////////////////////////////////////////////////
void __attribute__((naked)) _exit(int code)
{
    (void)code;

    for (;;)
        ;
}
