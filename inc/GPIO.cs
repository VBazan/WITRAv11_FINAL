using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

/// <summary>
/// Wrapper class for the GPIOLib.dll
/// Make sure that the DLL is avilible on the device.
/// </summary>
class GPIO
{
    //GPIO directions
    public const Int32 DIR_IN = 0;
    public const Int32 DIR_OUT = 1;
    //GPIO EDGE dedection
    public const Int32 EDGE_RISING = 1;
    public const Int32 EDGE_FALLING = 2;
    public const Int32 EDGE_BOTH = 3;


    [StructLayout(LayoutKind.Sequential)]
    public struct PIN_INSTANCE{
	    public byte inst1; //:  8; // instance 1
        public byte inst2; //:  8; // instance 2
      //DWORD res   : 16; // reserved
    }

    public const UInt32 TEGRA_ALTFN_GPIO = 4294967295;

    /// <summary>
    /// Init the GPIO unit
    /// </summary>
    [DllImport("GPIOLib.dll")]
    extern public static void InitGPIOLib(void);

    /// <summary>
    /// Deinit the GPIO unit
    /// </summary>
    [DllImport("GPIOLib.dll")]
    extern public static void DeInitGPIOLib(void);

    #region older GPIO functions, we recommend to use the Pin Functon instead of the GPIO Functions
    
    /// <summary>
    /// Get GPIO Direction
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <returns>GPIO Direction</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Int32 GetGPIODir(Int32 gpioNum);

    /// <summary>
    /// Get SODIMM Direction
    /// </summary>
    /// <param name="pinNum">SODIMM Number</param>
    /// <returns>GPIO Direction</returns>
    [DllImport("GPIOLib.dll")]
    extern public static UInt32 GetPinDir(UInt32 pinNum);

    /// <summary>
    /// Set GPIO Direction
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <param name="dirOut">GPIO Direction</param>
    [DllImport("GPIOLib.dll")]
    extern public static void SetGPIODir(Int32 gpioNum, Boolean dirOut);

    /// <summary>
    /// Set SODIMM Direction
    /// </summary>
    /// <param name="pinNum">SODIMM Number</param>
    /// <param name="dirOut">SODIMM Direction</param>
    [DllImport("GPIOLib.dll")]
    extern public static void SetPinDir(UInt32 pinNum, Boolean dirOut);

    /// <summary>
    /// Get Alternative Function of GPIO 
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <returns>Alternative Function of GPIO</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Int32 GetGPIOAltFn(Int32 gpioNum);

    /// <summary>
    /// Get Alternative Function of SODIM Pin
    /// </summary>
    /// <param name="pinNum">GPIO Number</param>
    /// <returns>Alternative Function of SODIMM Pin</returns>
    [DllImport("GPIOLib.dll")]
    extern public static UInt32 GetPinAltFn(UInt32 pinNum);

    /// <summary>
    /// Set Alternative Function of GPIO Pin
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <param name="altFn">Alternative Function</param>
    /// <param name="dirOut">GPIO Direction</param>
    [DllImport("GPIOLib.dll")]
    extern public static void SetGPIOAltFn(Int32 gpioNum, Int32 altFn, Boolean dirOut);


    /// <summary>
    /// Get GPIO Level
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <returns>GPIO Level</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Boolean GetGPIOLevel(Int32 gpioNum);

    /// <summary>
    /// Get Pin Level
    /// </summary>
    /// <param name="pinNum">SODIMM Pin Number</param>
    /// <returns>SODIMM Pin Level</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Boolean GetPinLevel(UInt32 pinNum);

    /// <summary>
    /// Set GPIO Level
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <param name="val">Level</param>
    [DllImport("GPIOLib.dll")]
    extern public static void SetGPIOLevel(Int32 gpioNum, Int32 val);

    /// <summary>
    /// Set SODIMM Pin Level
    /// </summary>
    /// <param name="pinNum">SODIMM Pin Number</param>
    /// <param name="val">Level</param>
    [DllImport("GPIOLib.dll")]
    extern public static void SetPinLevel(UInt32 pinNum, UInt32 val);

    /// <summary>
    /// Get GPIO Edge Detect
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <returns>Edge dedection</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Int32 GetGPIOEdgeDetect(Int32 gpioNum);

    /// <summary>
    /// Set GPIO Edge Detect
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <param name="edge">Edge Detect</param>
    [DllImport("GPIOLib.dll")]
    extern public static void SetGPIOEdgeDetect(Int32 gpioNum, Int32 edge);

    /// <summary>
    /// Get GPIO Edge Status
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <returns>GPIO Edge Status</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Boolean GetGPIOEdgeStatus(Int32 gpioNum);

    /// <summary>
    /// Clear GPIO Edge Status
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    [DllImport("GPIOLib.dll")]
    extern public static void ClearGPIOEdgeStatus(Int32 gpioNum);


    #endregion

    /// <summary>
    /// GetPinLevel
    /// </summary>
    /// <param name="pinNum">SODIMM Pin Number</param>
    [DllImport("GPIOLib.dll")]
    extern public static Boolean GetPinLevel(Int32 pinNum);


    /// <summary>
    /// SetPinLevel
    /// </summary>
    /// <param name="pinNum">SODIMM Pin Number</param>
    /// <param name="val">Value</param>
    [DllImport("GPIOLib.dll")]
    extern public static Int32 SetPinLevel(Int32 pinNum, Int32 val);

    /// <summary>
    /// GetPinAltFn
    /// </summary>
    /// <param name="pinNum">SODIMM Pin Number</param>
    [DllImport("GPIOLib.dll")]
    extern public static Int32 GetPinAltFn(Int32 pinNum);


    /// <summary>
    /// SetPinAltFn
    /// </summary>
    /// <param name="pinNum">SODIMM Pin Number</param>
    /// <param name="altFn">Alternativ Funktion, -1 on does always mean GPIO Function. (You need to use -1 on the Tegra if you like to set a pin as GPIO)</param> 
    /// <param name="dirOut">GPIO Direction [true:Output  false:Input]</param>
    [DllImport("GPIOLib.dll")]
    extern public static Int32 SetPinAltFn(Int32 pinNum, Int32 altFn, Int32 dirOut);


    /// <summary>
    /// GetGPIOFromPin
    /// </summary>
    /// <param name="pinNum">SODIMM Pin Number</param>
    /// <param name="extensionConnector">FALSE:SODIMM pin   TRUE:Extension connector</param> 
    /// <param name="gpio">gpio instances</param>
    [DllImport("GPIOLib.dll")]
    extern public static Int32 GetGPIOFromPin(Int32 pinNum, Int32 extensionConnector, out PIN_INSTANCE gpio);


    /// <summary>
    /// GPIOGetLibVersion
    /// </summary>
    [DllImport("GPIOLib.dll")]
    extern public static void GPIOGetLibVersion(out Int32 pVerMaj, out Int32 pVerMin, out Int32 pBuild);


    #region Tegra Specific Functions
    /// <summary>
    /// Set the Tristate settign of a GPIO
    /// Please note this affect the whole pin group!
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <param name="tristate">Pull State: 0: nothing, 1:pullup, 2:pulldown</param>
    /// <returns>TRUE  success</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Boolean TegraSetTristatePinGroup(UInt32 gpioNum, Boolean tristate);

    /// <summary>
    /// Get the Tristate settign of a GPIO
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <returns>1: Tristate, 0: Active</returns>
    [DllImport("GPIOLib.dll")]
    extern public static UInt32 TegraGetTristatePinGroup(UInt32 gpioNum);

    /// <summary>
    /// Set the Pullup/down settign of a GPIO
    /// Please note this affect the whole pin group!
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <param name="pullState">Pull State (0: nothing, 1:pullup, 2:pulldown)</param>
    /// <returns>TRUE success</returns>
    [DllImport("GPIOLib.dll")]
    extern public static Boolean TegraSetPullStatePinGroup(UInt32 gpioNum);

    /// <summary>
    /// Get the Pullup/down settign of a GPIO
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <returns>Pull status (0: nothing, 1:pullup, 2:pulldown)</returns>
    [DllImport("GPIOLib.dll")]
    extern public static UInt32 TegraGetPullStatePinGroup(UInt32 gpioNum);

    /// <summary>
    /// Get the Pin Group Name of a GPIO
    /// </summary>
    /// <param name="gpioNum">GPIO Number</param>
    /// <param name="groupName">[out]*groupName	Name of the Pin group max TEGRA_MAX_GROUPNAME characters.</param>
    /// <returns>TRUE  success</returns>
    [DllImport("GPIOLib.dll")]
    extern public static UInt32 TegraGetPinGroupName(UInt32 gpioNum, Char groupName);

    #endregion
}
