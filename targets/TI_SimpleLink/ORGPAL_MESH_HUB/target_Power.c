//
// Copyright (c) 2020 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#include <driverlib/sys_ctrl.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <nanoHAL_Power.h>

void CPU_Reset()
{
    SysCtrlSystemReset();
};

// strong implementation of CPU_SetPowerModeTarget
void CPU_SetPowerModeTarget(PowerLevel_type powerLevel)
{
    (void)powerLevel;

    // disable UART pins and ADC
    PIN_Config BoardGpioInitTable[] = {
        12 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
        13 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
        24 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
        PIN_TERMINATE};

    PIN_init(BoardGpioInitTable);
}
