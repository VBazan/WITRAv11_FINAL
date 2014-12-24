using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;
using System.ComponentModel;

namespace TestCsharp
{
    class IOCTL
    {
        /// <summary>
        /// Provides the ability to soft reset a device.
        /// </summary>
        // Control Code flags
        public const uint FILE_DEVICE_HAL = 0x00000101;
        public const uint FILE_DEVICE_CONSOLE = 0x00000102;
        public const uint FILE_DEVICE_PSL = 0x00000103;
        public const uint METHOD_BUFFERED = 0;
        public const uint METHOD_IN_DIRECT = 1;
        public const uint METHOD_OUT_DIRECT = 2;
        public const uint METHOD_NEITHER = 3;
        public const uint FILE_ANY_ACCESS = 0;
        public const uint FILE_READ_ACCESS = 0x0001;
        public const uint FILE_WRITE_ACCESS = 0x0002;

        /// <summary>
        /// Create a control code from the specified device, function, method, and
        /// access flags.
        /// </summary>
        /// <param name="DeviceType">Device type flag</param>
        /// <param name="Function">Function number</param>
        /// <param name="Method">Method flag</param>
        /// <param name="Access">Access flag</param>
        /// <returns>A control code based on the specified parameters</returns>
        public static uint CTL_CODE(uint DeviceType, uint Function, uint Method, uint Access)
        {
            return ((DeviceType << 16) | (Access << 14) | (Function << 2) | Method);
        }

        /// <summary>
        /// This function provides the kernel with a generic I/O control for
        /// carrying out I/O operations.
        /// </summary>
        /// <param name="dwIoControlCode">I/O control code, which should support the
        /// OAL I/O controls. For a list of these I/O controls, see Supported
        /// OAL APIs.</param>
        /// <param name="lpInBuf">Pointer to the input buffer.</param>
        /// <param name="nInBufSize">Size, in bytes, of lpInBuf.</param>
        /// <param name="lpOutBuf">Pointer to the output buffer.</param>
        /// <param name="nOutBufSize">Maximum number of bytes that can be returned in
        /// lpOutBuf.</param>
        /// <param name="lpBytesReturned">Address of a DWORD that receives the size,
        /// in bytes, of the data returned.</param>
        /// <returns>TRUE indicates success; FALSE indicates failure.</returns>
        [DllImport("Coredll.dll")]
        //public extern static uint KernelIoControl
        //(
        //    uint dwIoControlCode,
        //    IntPtr lpInBuf,
        //    uint nInBufSize,
        //    IntPtr lpOutBuf,
        //    uint nOutBufSize,
        //    ref uint lpBytesReturned
        //);
        public static extern bool KernelIoControl([MarshalAs(UnmanagedType.U4)]uint dwIOControlCode, [MarshalAs(UnmanagedType.LPArray)] byte[] lpInBuf, [MarshalAs(UnmanagedType.U4)] uint nInBufSize, [MarshalAs(UnmanagedType.LPArray)] byte[] lpOutBuf, [MarshalAs(UnmanagedType.U4)] uint nOutBufSize, ref uint lpBytesReturned);



        public static void TestProc()
        {
            uint IOCTL_HAL_REBOOT = CTL_CODE(FILE_DEVICE_HAL, 15, METHOD_BUFFERED, FILE_ANY_ACCESS);
            uint IOCTL_HAL_GPIO2IRQ = CTL_CODE(FILE_DEVICE_HAL, 2048, METHOD_BUFFERED, FILE_ANY_ACCESS);
            uint IOCTL_HAL_IRQ2GPIO = CTL_CODE(FILE_DEVICE_HAL, 2049, METHOD_BUFFERED, FILE_ANY_ACCESS);
            uint IOCTL_HAL_IRQEDGE = CTL_CODE(FILE_DEVICE_HAL, 2050, METHOD_BUFFERED, FILE_ANY_ACCESS);
            uint IOCTL_HAL_REQUEST_SYSINTR = CTL_CODE(FILE_DEVICE_HAL, 38, METHOD_BUFFERED, FILE_ANY_ACCESS);
            uint IOCTL_HAL_RELEASE_SYSINTR = CTL_CODE(FILE_DEVICE_HAL, 54, METHOD_BUFFERED, FILE_ANY_ACCESS);

            //uint IOCTL_HAL_GPIO2IRQXX = ((FILE_DEVICE_HAL << 16) | (((uint)2048) << 14) | (METHOD_BUFFERED << 2) | FILE_ANY_ACCESS);
            uint IOCTL_HAL_GPIO2IRQXX = (FILE_DEVICE_HAL << 16); 
             IOCTL_HAL_GPIO2IRQXX   |= (((uint)2048) << 14);
             IOCTL_HAL_GPIO2IRQXX |=(METHOD_BUFFERED << 2);
            //uint bytesReturned = 0;
            //uint dwIrq=0;
            //uint dwGpio = 36;
            //uint test;
            //IntPtr pdwGpio= new IntPtr(dwGpio);
            //IntPtr pdwIrq = new IntPtr(dwIrq);
            //uint[] argin = { 32 };
            //uint[] argout = { 0 };
            ////KernelIoControl(IOCTL_HAL_GPIO2IRQ, pdwGpio, 1, pdwIrq, 1, ref bytesReturned);

            uint dwEdge = 0, dwGpio = 0;
            uint dwIrq = 0, dwSysIntr = 0;
            uint dwBytesReturned = 0;
            byte[] argin = { 0 };
            byte[] argout = { 0 };
            int err;

            // Get the IRQ # for this GPIO
            dwGpio = 36;
            argin[0] = (byte)dwGpio;
            if (!KernelIoControl(IOCTL_HAL_GPIO2IRQ, argin, 1, argout, 1, ref dwBytesReturned))
            {
                throw new Win32Exception(err = Marshal.GetLastWin32Error());
            }


            

        }

    }
}
