using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.ComponentModel;

class CoreDll
{
    public const uint WAIT_OBJECT_0 = 0;
    public const uint WAIT_TIMEOUT = 0x00000102;
    public const uint INFINITE = 0xFFFFFFFF;
    public const uint GPIO_EDGE_RISING = 1, GPIO_EDGE_FALLING = 2;
    const uint FILE_DEVICE_HAL = 0x00000101;
    const uint METHOD_BUFFERED = 0;
    const uint FILE_ANY_ACCESS = 0;
    public const uint IOCTL_HAL_GPIO2IRQ = FILE_DEVICE_HAL << 16 | FILE_ANY_ACCESS << 14 | 2048 << 2 | METHOD_BUFFERED;
    public const uint IOCTL_HAL_IRQ2GPIO = FILE_DEVICE_HAL << 16 | FILE_ANY_ACCESS << 14 | 2049 << 2 | METHOD_BUFFERED;
    public const uint IOCTL_HAL_IRQEDGE = FILE_DEVICE_HAL << 16 | FILE_ANY_ACCESS << 14 | 2050 << 2 | METHOD_BUFFERED;
    public const uint IOCTL_HAL_REQUEST_SYSINTR = FILE_DEVICE_HAL << 16 | FILE_ANY_ACCESS << 14 | 38 << 2 | METHOD_BUFFERED;
    public const uint IOCTL_HAL_RELEASE_SYSINTR = FILE_DEVICE_HAL << 16 | FILE_ANY_ACCESS << 14 | 54 << 2 | METHOD_BUFFERED;

    [DllImport("coredll.dll", SetLastError = true)]
    public static extern bool KernelIoControl(uint dwIOControlCode, ref uint lpInBuf, uint nInBufSize, ref uint lpOutBuf, uint nOutBufSize, ref uint lpBytesReturned);

    [DllImport("coredll.dll", SetLastError = true)]
    public static extern bool KernelIoControl(uint dwIOControlCode, ref uint lpInBuf, uint nInBufSize, System.IntPtr lpOutBuf, uint nOutBufSize, ref uint lpBytesReturned);

    [DllImport("coredll.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    public static extern uint CreateEvent(uint lpEventAttributes, bool bManualReset, bool bInitialState, string lpName);

    [DllImport("coredll.dll", SetLastError = true)]
    public static extern bool InterruptInitialize(uint idInt, uint hEvent, uint pvData, uint cbData);

    [DllImport("coredll.dll", SetLastError = true)]
    public static extern bool InterruptDisable(uint idInt);

    [DllImport("coredll.dll", SetLastError = true)]
    public static extern uint WaitForSingleObject(uint Handle, uint Wait);

    [DllImport("coredll.dll", SetLastError = true)]
    public static extern void InterruptDone(uint idInt);


    public static void testFn()
    {
        uint hEvent;

        uint dwEdge = 0 , dwGpio = 0 ;
        uint dwIrq = 0, dwSysIntr = 0;
        uint dwOutBuf = 0;
        uint dwBytesReturned=0;
        int err;
        bool ret;
        String retText;

        // Get the IRQ # for this GPIO
        dwGpio = 36;
        ret = CoreDll.KernelIoControl(CoreDll.IOCTL_HAL_GPIO2IRQ, ref dwGpio, sizeof(uint), ref dwIrq, sizeof(uint), ref dwBytesReturned);
        
        // Configure for an interrupt on the rising edge            
        dwEdge = CoreDll.GPIO_EDGE_RISING;
        if(!CoreDll.KernelIoControl(CoreDll.IOCTL_HAL_IRQEDGE, ref dwIrq, 1, ref dwEdge, 1, ref dwBytesReturned)) throw new Win32Exception(err = Marshal.GetLastWin32Error());

        // Create an event associated with this interrupt
        hEvent = CoreDll.CreateEvent(0, false, false, null);
        retText = hEvent.ToString();

        // Register the event
        ret = CoreDll.KernelIoControl(CoreDll.IOCTL_HAL_REQUEST_SYSINTR, ref dwIrq, sizeof(uint), ref dwSysIntr, sizeof(uint), ref dwBytesReturned);

        // Initialize the interrupt
        ret = CoreDll.InterruptInitialize(dwSysIntr, hEvent, 0, 0);


        // Clear the interrupt if there was one pending
        CoreDll.InterruptDone(dwSysIntr);

        // Wait for something to happen!
        bool Exit = false;
        while (!Exit)
        {
            if (CoreDll.WaitForSingleObject(hEvent, CoreDll.INFINITE) == CoreDll.WAIT_OBJECT_0)
            {
                CoreDll.InterruptDone(dwSysIntr);
                if(MessageBox.Show("Interrupt!", "Wait again", MessageBoxButtons.YesNo, MessageBoxIcon.Question, MessageBoxDefaultButton.Button1)==DialogResult.No)
                //If( DialogResult.No == MessageBox.Show("Interrupt!","Wait again",MessageBoxButtons.YesNo,MessageBoxIcon.Question,MessageBoxDefaultButton.Button1))
                {
                    Exit = true;
                }
                
            }
        }

        // Disable any previously registered event
        CoreDll.InterruptDisable(dwSysIntr);
        CoreDll.KernelIoControl(CoreDll.IOCTL_HAL_RELEASE_SYSINTR, ref dwSysIntr, sizeof(uint), System.IntPtr.Zero, 0, ref dwBytesReturned);
        //if (!CoreDllWrapper.KernelIoControl(CoreDllWrapper.IOCTL_HAL_RELEASE_SYSINTR, ref dwSysIntr, sizeof(uint), System.IntPtr.Zero, 0, ref dwBytesReturned)) throw new Win32Exception(err = Marshal.GetLastWin32Error());

    }

}

