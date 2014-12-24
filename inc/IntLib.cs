using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

class IntLib
{
    //Win CE 5.0/6.0/7.0 compatible version of InterruptInitialize()
    [DllImport("IntLib.dll", SetLastError = true)]
    public static extern Int32 InterruptInitializeCompat(Int32 idInt, UInt32 hEvent, Int32 pvData, Int32 cbData);

    //Win CE 5.0/6.0/7.0 compatible version of InterruptDisable()
    [DllImport("IntLib.dll", SetLastError = true)]
    public static extern void InterruptDisableCompat(Int32 idInt);

    //Win CE 5.0/6.0/7.0 compatible version of Interrupt()
    [DllImport("IntLib.dll", SetLastError = true)]
    public static extern void InterruptDoneCompat(Int32 idInt);

    /*
    Get the SYSIntr Number for the IRQ
    (Replace the KernelIOControl: IOCTL_HAL_REQUEST_SYSINTR)
    dwIrq: IRQ Number
    return: SysInt Number
    */
    [DllImport("IntLib.dll", SetLastError = true)]
    public static extern Int32 RequestSysInterrupt(Int32 dwIrq);


    /*
    Releases the SysInterrupt
    (Replace the KernelIOControl: IOCTL_HAL_RELEASE_SYSINTR)
    dwSysIntr: SysInt Number
    */
    [DllImport("IntLib.dll", SetLastError = true)]
    public static extern Int32 ReleaseSysIntr(Int32 dwSysIntr);


    /*
    Get an IRQ for an specific GPIO
    dwGpio: GPIO Number
    return: IRW Number
    */
    [DllImport("IntLib.dll", SetLastError = true)]
    public static extern Int32 GetGPIOIrq(Int32 dwGpio);

    //Set GPIO Interrupt Edge dedect. See also the defines
    [DllImport("IntLib.dll", SetLastError = true)]
    public static extern Int32 SetGPIOIrqEdge(int dwGpio, int dwEdge);

    /*
    Description:      Returns the library Version
    Parameter:
    Out-Parameter:
    - pVerMaj:          Returns the major version number. Set this parameter to NULL if not required.
    - pVerMin:          Returns the minor version number. Set this parameter to NULL if not required.
    - pBuild:           Returns the build number.         Set this parameter to NULL if not required.
    Return Value:		No return value.
    */
    public static extern void INTGetLibVersion(out Int32 pVerMaj, out Int32 pVerMin, out Int32 pBuild);

}

