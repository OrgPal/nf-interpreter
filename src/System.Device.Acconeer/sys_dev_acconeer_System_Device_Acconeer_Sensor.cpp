//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "sys_dev_acconeer.h"

// sensor reservation: 1 bit per sensor, 0 indexed, 0 = not reserved, 1 = reserved
// static uint8_t sensorReserved;

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::_ctor___VOID__U4( CLR_RT_StackFrame &stack )
{
    NANOCLR_HEADER();

    (void)stack;
    
	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
	}

    NANOCLR_NOCLEANUP();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::PerformCalibration___BOOLEAN__I4( CLR_RT_StackFrame &stack )
{
    NANOCLR_HEADER();

    NANOCLR_SET_AND_LEAVE(stack.NotImplementedStub());

    NANOCLR_NOCLEANUP();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeInit___VOID( CLR_RT_StackFrame &stack )
{
    NANOCLR_HEADER();

    NANOCLR_SET_AND_LEAVE(stack.NotImplementedStub());

    NANOCLR_NOCLEANUP();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeDeInit___VOID( CLR_RT_StackFrame &stack )
{
    NANOCLR_HEADER();

    NANOCLR_SET_AND_LEAVE(stack.NotImplementedStub());

    NANOCLR_NOCLEANUP();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeSetEnable___VOID__BOOLEAN( CLR_RT_StackFrame &stack )
{
    NANOCLR_HEADER();

    NANOCLR_SET_AND_LEAVE(stack.NotImplementedStub());

    NANOCLR_NOCLEANUP();
}
