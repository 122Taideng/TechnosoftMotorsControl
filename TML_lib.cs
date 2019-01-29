using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace TML
{
    public class TMLLib
    {
        //supported CAN protocols
        public const Byte PROTOCOL_TMLCAN = 0x00;    //use TMLCAN protocol (default, 29-bit identifiers)
        public const Byte PROTOCOL_TECHNOCAN = 0x80; //use TechnoCAN protocol (11-bit identifiers)
        public const Byte PROTOCOL_MASK = 0x80;      //this bits are used for specifying CAN protocol through nChannelType param of MSK_OpenComm function

        /***** supported CAN devices *****************************
        CHANNEL_IXXAT_CAN - see http://www.ixxat.com
        CHANNEL_SYS_TEC_USBCAN - see www.systec-electronic.com
        CHANNEL_ESD_CAN - see http://www.esd-electronics.com
        CHANNEL_PEAK_SYS_PCAN_* - see http://www.peak-system.com
        **********************************************************/
        public const Byte CHANNEL_RS232 = 0;
        public const Byte CHANNEL_RS485 = 1;
        public const Byte CHANNEL_IXXAT_CAN = 2;
        public const Byte CHANNEL_SYS_TEC_USBCAN = 3;
        public const Byte CHANNEL_PEAK_SYS_PCAN_PCI = 4;
        public const Byte CHANNEL_ESD_CAN = 5;
        public const Byte CHANNEL_PEAK_SYS_PCAN_ISA = 6;
        public const Byte CHANNEL_PEAK_SYS_PCAN_PC104 = CHANNEL_PEAK_SYS_PCAN_ISA; //Same with PCAN_ISA
        public const Byte CHANNEL_PEAK_SYS_PCAN_USB = 7;
        public const Byte CHANNEL_PEAK_SYS_PCAN_DONGLE = 8;
        public const Byte CHANNEL_VIRTUAL_SERIAL = 15;
        public const Byte CHANNEL_XPORT_IP = 16;

        /*Constant used for host ID*/
        public const Byte HOST_ID = 0;

        /*Constants used as values for 'Logger' parameters*/
        public const Byte LOGGER_SLOW = 1;
        public const Byte LOGGER_FAST = 2;

        /*Constants used as values for 'MoveMoment' parameters*/
        public const Int16 UPDATE_NONE = -1;
        public const Int16 UPDATE_ON_EVENT = 0;
        public const Int16 UPDATE_IMMEDIATE = 1;

        /*Constants used for 'ReferenceType' parameters*/
        public const Int16 REFERENCE_POSITION = 0;
        public const Int16 REFERENCE_SPEED = 1;
        public const Int16 REFERENCE_TORQUE = 2;
        public const Int16 REFERENCE_VOLTAGE = 3;

        /*Constants used for EnableSuperposition*/
        public const Int16 SUPERPOS_DISABLE = -1;
        public const Int16 SUPERPOS_NONE = 0;
        public const Int16 SUPERPOS_ENABLE = 1;

        /*Constants used for PositionType*/
        public const Int16 ABSOLUTE_POSITION = 0;
        public const Int16 RELATIVE_POSITION = 1;

        /*Constants used for EnableSlave*/
        public const Int16 SLAVE_NONE = 0;
        public const Int16 SLAVE_COMMUNICATION_CHANNEL = 1;
        public const Int16 SLAVE_2ND_ENCODER = 2;

        /*Constants used for ReferenceBase*/
        public const Int16 FROM_MEASURE = 0;
        public const Int16 FROM_REFERENCE = 1;

        /*Constants used for DecelerationType*/
        public const Int16 S_CURVE_SPEED_PROFILE = 0;
        public const Int16 TRAPEZOIDAL_SPEED_PROFILE = 1;

        /*Constants used for IOState*/
        public const byte IO_HIGH = 1;
        public const byte IO_LOW = 0;

        /*Constants used for TransitionType*/
        public const Int16 TRANSITION_HIGH_TO_LOW = -1;
        public const Int16 TRANSITION_DISABLE = 0;
        public const Int16 TRANSITION_LOW_TO_HIGH = 1;

        /*Constants used for IndexType*/
        public const Int16 INDEX_1 = 1;
        public const Int16 INDEX_2 = 2;

        /*Constants used for LSWType*/
        public const Int16 LSW_NEGATIVE = -1;
        public const Int16 LSW_POSITIVE = 1;

        /*Constants used for TS_Power; to activate/deactivate teh PWM commands*/
        public const Boolean POWER_ON = true;
        public const Boolean POWER_OFF = false;

        /*Constants used as inputs parameters of the I/O functions*/
        public const Byte INPUT_0 = 0;
        public const Byte INPUT_1 = 1;
        public const Byte INPUT_2 = 2;
        public const Byte INPUT_3 = 3;
        public const Byte INPUT_4 = 4;
        public const Byte INPUT_5 = 5;
        public const Byte INPUT_6 = 6;
        public const Byte INPUT_7 = 7;
        public const Byte INPUT_8 = 8;
        public const Byte INPUT_9 = 9;
        public const Byte INPUT_10 = 10;
        public const Byte INPUT_11 = 11;
        public const Byte INPUT_12 = 12;
        public const Byte INPUT_13 = 13;
        public const Byte INPUT_14 = 14;
        public const Byte INPUT_15 = 15;
        public const Byte INPUT_16 = 16;
        public const Byte INPUT_17 = 17;
        public const Byte INPUT_18 = 18;
        public const Byte INPUT_19 = 19;
        public const Byte INPUT_20 = 20;
        public const Byte INPUT_21 = 21;
        public const Byte INPUT_22 = 22;
        public const Byte INPUT_23 = 23;
        public const Byte INPUT_24 = 24;
        public const Byte INPUT_25 = 25;
        public const Byte INPUT_26 = 26;
        public const Byte INPUT_27 = 27;
        public const Byte INPUT_28 = 28;
        public const Byte INPUT_29 = 29;
        public const Byte INPUT_30 = 30;
        public const Byte INPUT_31 = 31;
        public const Byte INPUT_32 = 32;
        public const Byte INPUT_33 = 33;
        public const Byte INPUT_34 = 34;
        public const Byte INPUT_35 = 35;
        public const Byte INPUT_36 = 36;
        public const Byte INPUT_37 = 37;
        public const Byte INPUT_38 = 38;
        public const Byte INPUT_39 = 39;

        public const Byte OUTPUT_0 = 0;
        public const Byte OUTPUT_1 = 1;
        public const Byte OUTPUT_2 = 2;
        public const Byte OUTPUT_3 = 3;
        public const Byte OUTPUT_4 = 4;
        public const Byte OUTPUT_5 = 5;
        public const Byte OUTPUT_6 = 6;
        public const Byte OUTPUT_7 = 7;
        public const Byte OUTPUT_8 = 8;
        public const Byte OUTPUT_9 = 9;
        public const Byte OUTPUT_10 = 10;
        public const Byte OUTPUT_11 = 11;
        public const Byte OUTPUT_12 = 12;
        public const Byte OUTPUT_13 = 13;
        public const Byte OUTPUT_14 = 14;
        public const Byte OUTPUT_15 = 15;
        public const Byte OUTPUT_16 = 16;
        public const Byte OUTPUT_17 = 17;
        public const Byte OUTPUT_18 = 18;
        public const Byte OUTPUT_19 = 19;
        public const Byte OUTPUT_20 = 20;
        public const Byte OUTPUT_21 = 21;
        public const Byte OUTPUT_22 = 22;
        public const Byte OUTPUT_23 = 23;
        public const Byte OUTPUT_24 = 24;
        public const Byte OUTPUT_25 = 25;
        public const Byte OUTPUT_26 = 26;
        public const Byte OUTPUT_27 = 27;
        public const Byte OUTPUT_28 = 28;
        public const Byte OUTPUT_29 = 29;
        public const Byte OUTPUT_30 = 30;
        public const Byte OUTPUT_31 = 31;
        public const Byte OUTPUT_32 = 32;
        public const Byte OUTPUT_33 = 33;
        public const Byte OUTPUT_34 = 34;
        public const Byte OUTPUT_35 = 35;
        public const Byte OUTPUT_36 = 36;
        public const Byte OUTPUT_37 = 37;
        public const Byte OUTPUT_38 = 38;
        public const Byte OUTPUT_39 = 39;

        /*Constants used for the register for function TS_ReadStatus*/
        public const Int16 REG_MCR = 0;
        public const Int16 REG_MSR = 1;
        public const Int16 REG_ISR = 2;
        public const Int16 REG_SRL = 3;
        public const Int16 REG_SRH = 4;
        public const Int16 REG_MER = 5;

        /*Constants used to select or set the group*/
        public const Byte GROUP_0 = 0;
        public const Byte GROUP_1 = 1;
        public const Byte GROUP_2 = 2;
        public const Byte GROUP_3 = 3;
        public const Byte GROUP_4 = 4;
        public const Byte GROUP_5 = 5;
        public const Byte GROUP_6 = 6;
        public const Byte GROUP_7 = 7;
        public const Byte GROUP_8 = 8;

        /*Special parameter values*/
        public const Int32 FULL_RANGE = 0;
        public const Int32 NO_VARIATION = 0;

        /****************************************************************************
        Callback function used by client application for handling unsolicited
        messages which this driver receives in unexpected places
        ****************************************************************************/
        public delegate void pfnCallbackRecvDriveMsg(UInt16 wAxisID, UInt16 wAddress, Int32 Value);

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern String TS_GetLastErrorText();
        /*******************************************************************************************
         Function: Returns a text related to the last occurred error when one of the library functions
                   was called.
         Input arguments:
	        -
         Output arguments:
	        return: A text related to the last occurred error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern void TS_Basic_GetLastErrorText(StringBuilder strError, Int32 nBuffSize);
        /*******************************************************************************************
         Function: Updates strError with a text related to the last occurred error when one of the library functions
                   was called.
         Input arguments:
	        strError - string with error text
            nBuffSize - number of chars in strError
         Output arguments:
	        -
        *******************************************************************************************/



        /*******************************************************************/
        /*******************Parametrization*********************************/
        /*******************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Int32 TS_LoadSetup(String setupPath);
        /*******************************************************************************************
         Function: Load setup information from a zip archive or a directory containing setup.cfg and variables.cfg files.
         Input arguments:
	        setupPath:		path to the zip archive or directory that contains setup.cfg and variables.cfg of the given setup 
         Output arguments:
	        return:		>=0 index of the loaded setup; -1 if error
        *******************************************************************************************/

        /*******************************************************************/
        /******************* Communication channels ************************/
        /*******************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Int32 TS_OpenChannel(String pszDevName, Byte btType, Byte nHostID, UInt32 baudrate);
        /*******************************************************************************************
         Function: Open a communication channel.
         Input arguments:
	        pszDevName:	Number of the serial channel to be open (for serial ports: "COM1", "COM2", ...; for CAN devices: "1", "2", ..)
	        btType: channel type (CHANNEL_*) with an optional protocol (PROTOCOL_*, default is PROTOCOL_TMLCAN)
	        nHostID: Is the address of your PC computer. A value between 1 and 255
			        For RS232: axis ID of the drive connected to the PC serial port (usually 255)
			        For RS485 or CAN devices: must be an unused axis ID! It is the address of your PC computer on
						        the RS485 network.
			        For XPORT: "IP:port"
	        BaudRate:	Baud rate 
			        serial ports: 9600, 19200, 38400, 56000 or 115200
			        CAN devices: 125000, 250000, 500000, 1000000
         Output arguments:
	        return:		channel's file descriptor or -1 if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SelectChannel(Int32 fd);
        /*******************************************************************************************
         Function: Select active communication channel. If you use only one channel there is no need to call this function.
         Input arguments:
	        fd: channel file descriptor (-1 means selected communication channel)
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern void TS_CloseChannel(Int32 fd);
        /*******************************************************************************************
         Function: Close the communication channel.
         Input arguments:
          fd: channel file descriptor (-1 means selected communication channel)
        *******************************************************************************************/

        /*******************************************************************/
        /*******************Drive Administration ***************************/
        /*******************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetupAxis(Byte axisID, Int32 idxSetup);
        /*******************************************************************************************
         Function: Select setup configuration for the drive with axis ID.
         Input arguments:
	        axisID:		axis ID. It must be a value between 1 and 255; 
	        idxSetup:	Index of previously loaded setup, 
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SelectAxis(Byte axisID);
        /*******************************************************************************************
         Function: Selects the active axis.
         Input arguments:
	        axisID:		The ID of the axis to become the active one. It must be a value between 1 and 255; 
					        For RS485/CAN communication, this value must be different than nHostID parameter 
					        defined at TS_OpenChannel function call.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetupGroup(Byte groupID, Int32 idxSetup);
        /*******************************************************************************************
         Function: Select setup configuration for the drives within group.
         Input arguments:
	        groupID:		group ID. It must be a value between 1 and 8; 
	        idxSetup:		Index of previously loaded setup, 
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SelectGroup(Byte groupID);
        /*******************************************************************************************
         Function: Selects the active group.
         Input arguments:
	        groupID:		The ID of the group of axes to become the active ones. It must be a value 
					        between 1 and 8.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetupBroadcast(Int32 idxSetup);
        /*******************************************************************************************
         Function: Select setup configuration for all drives on the active channel.
         Input arguments:
	        idxSetup:		Index of previously loaded setup, 
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SelectBroadcast();
        /*******************************************************************************************
         Function: Selects all axis on the active channel. 
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_Reset();
        /*******************************************************************************************
         Function: Resets selected drives.
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_ResetFault();
        /*******************************************************************************************
         Function: This function clears most of the errors bits from Motion Error Register (MER).
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_Power(Boolean Enable);
        /*******************************************************************************************
         Function: Controls the power stage (ON/OFF).
         Input arguments:
	        Enable:		TRUE -> Power ON the drive; FALSE -> Power OFF the drive
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_ReadStatus(Int16 SelIndex, out UInt16 Status);
        /*******************************************************************************************
         Function: Returns drive status information.
         Input arguments:
	        SelIndex:	
		        REG_MCR -> read MCR register
		        REG_MSR -> read MSR register
		        REG_ISR -> read ISR register 
		        REG_SRL -> read SRL register 
		        REG_SRH -> read SRH register 
		        REG_MER -> read MER register 
         Output arguments:
	        Status:	drive status information (value of the selected register)
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_Save();
        /*******************************************************************************************
         Function: Saves actual values of all the parameters from the drive/motor working memory into 
                   the EEPROM setup table.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_UpdateImmediate();
        /*******************************************************************************************
         Function: Update the motion mode immediately.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_UpdateOnEvent();
        /*******************************************************************************************
         Function: Update the motion mode on next event occurence.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetPosition(Int32 PosValue);
        /*******************************************************************************************
         Function: Set actual position value.
         Input arguments:
	        PosValue:	Value at which the position is set
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetCurrent(Int16 CrtValue);
        /*******************************************************************************************
         Function: Set actual current value.
         Input arguments:
	        CrtValue:	Value at which the motor current is set
					        REMARK: this command can be used only for step motor drives
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetTargetPositionToActual();
        /*******************************************************************************************
         Function: Set the target position value equal to the actual position value.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetVariableAddress(String pszName, out Int16 address);
        /*******************************************************************************************
         Function: Returns the variable address. The address is read from setup file
         Input arguments:
	        pszName:	Variable name
         Output arguments:
          address:	Variable address
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetIntVariable(String pszName, Int16 value);
        /*******************************************************************************************
         Function: Writes an integer type variable to the drive.
         Input arguments:
	        pszName:	Name of the variable
	        value:	Variable value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetIntVariable(String pszName, out Int16 value);
        /*******************************************************************************************
         Function: Reads an integer type variable from the drive.
         Input arguments:
	        pszName:	Name of the variable
         Output arguments:
	        value:	Variable value
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetLongVariable(String pszName, Int32 value);
        /*******************************************************************************************
         Function: Writes a Int32 integer type variable to the drive.
         Input arguments:
	        pszName:	Name of the variable
	        value:	Variable value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetLongVariable(String pszName, out Int32 value);
        /*******************************************************************************************
         Function: Reads a Int32 integer type variable from the drive.
         Input arguments:
	        pszName:	Name of the variable
         Output arguments:
	        value:	Variable value
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetFixedVariable(String pszName, Double value);
        /*******************************************************************************************
         Function: Writes a fixed point type variable to the drive.
         Input arguments:
	        pszName:	Name of the variable
	        value:	Variable value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetFixedVariable(String pszName, out Double value);
        /*******************************************************************************************
         Function: Reads a fixed point type variable from the drive.
         Input arguments:
	        pszName:	Name of the variable
         Output arguments:
	        value:	Variable value
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetBuffer(UInt16 address, UInt16[] arrayValues, UInt16 nSize);
        /*******************************************************************************************
         Function: Download a data buffer to the drive's memory. 
         Input arguments:
	        address:	Start address where to download the data buffer
	        arrayValues:	Buffer containing the data to be downloaded
	        nSize: the number of words to download
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetBuffer(UInt16 address, UInt16[] arrayValues, UInt16 nSize);
        /*******************************************************************************************
         Function: Upload a data buffer from the drive (get it from motion chip's memory). 
         Input arguments:
	        address:	Start address where from to upload the data buffer
	        arrayValues:	Buffer address where the uploaded data will be stored
	        nSize: the number of words to upload
         Output arguments:
	        arrayValues:	the uploaded data
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        /*******************************************************************/
        /*******************MOTION functions********************************/
        /*******************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_MoveAbsolute(Int32 AbsPosition, Double Speed, Double Acceleration, Int16 MoveMoment, Int16 ReferenceBase);
        /*******************************************************************************************
         Function: Move Absolute with trapezoidal speed profile. This function allows you to program a position profile 
                   with a trapezoidal shape of the speed.
         Input arguments:
	        AbsPosition: 	Absolute position reference value
	        Speed: 		Slew speed; if 0, use previously defined value
	        Acceleration: 	Acceleration  decceleration; if 0, use previously defined value
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_MoveRelative(Int32 RelPosition, Double Speed, Double Acceleration, Boolean IsAdditive, Int16 MoveMoment, Int16 ReferenceBase);
        /*******************************************************************************************
         Function: Move Relative with trapezoidal speed profile. This function allows you to program a position profile 
                   with a trapezoidal shape of the speed.
         Input arguments:
	        RelPosition: 	Relative position reference value
	        Speed: 		Slew speed; if 0, use previously defined value
	        Acceleration: 	Acceleration  decceleration; if 0, use previously defined value
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        IsAdditive:
		        TRUE -> Add the position increment to the position to reach set by the previous motion command
		        FALSE -> No position increment is added to the target position
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_MoveVelocity(Double Speed, Double Acceleration, Int16 MoveMoment, Int16 ReferenceBase);
        /*******************************************************************************************
         Function: Move at a given speed, with acceleration profile.
         Input arguments:
	        Speed: 		Jogging speed
	        Acceleration: 	Acceleration  decceleration; if 0, use previously defined value
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetAnalogueMoveExternal(Int16 ReferenceType, Boolean UpdateFast, Double LimitVariation, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Set Motion type as using an analogue external reference. 
         Input arguments:
	        ReferenceType:	
		        REFERENCE_POSITION -> external position reference
		        REFERENCE_SPEED -> external speed reference
		        REFERENCE_TORQUE -> external torque reference
		        REFERENCE_VOLTAGE -> external voltage reference
	        UpdateFast: 	
		        TRUE -> generate the torque reference in the fast control loop
		        FALSE -> generate the torque reference in the slow control loop
	        LimitVariation:
		        NO_VARIATION (0) -> the external reference is limited at the value set in the Drive Setup
		        A value which can be an acceleration or speed in function of the reference type.
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetDigitalMoveExternal(Boolean SetGearRatio, Int16 Denominator, Int16 Numerator, Double LimitVariation, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Set Motion type as using a digital external reference. This function is used only for Positioning.
         Input arguments:
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        LimitVariation:
		        NO_VARIATION (0) -> the external reference is limited at the value set in the Drive Setup
		        A value which can be an acceleration or speed in function of the reference type.
	        SetGearRatio: Set the gear parameters; if TRUE, following parameters are needed
	        Denumerator: Gear master ratio
	        Numerator: Gear slave ratio
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetOnlineMoveExternal(Int16 ReferenceType, Double LimitVariation, Double InitialValue, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Set Motion type as using an analogue external reference.
         Input arguments:
	        ReferenceType:	
		        REFERENCE_POSITION -> external position reference
		        REFERENCE_SPEED -> external speed reference
		        REFERENCE_TORQUE -> external torque reference
		        REFERENCE_VOLTAGE -> external voltage reference
	        LimitVariation:
		        NO_VARIATION (0) -> the external reference is limited at the value set in the Drive Setup
		        A value which can be an acceleration or speed in function of the reference type.
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        InitialValue: If non zero, set initial value of EREF
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_VoltageTestMode(Int16 MaxVoltage, Int16 IncrVoltage, Int16 Theta0, Int16 Dtheta, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Use voltage test mode. 
         Input arguments:
	        MaxVoltage:	Maximum test voltage value
	        IncrVoltage:	Voltage increment on each slow sampling period
	        Theta0:		Initial value of electrical angle value
			        Remark: used only for AC motors; set to 0 otherwise
	        Dtheta:		Electric angle increment on each slow sampling period
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_TorqueTestMode(Int16 MaxTorque, Int16 IncrTorque, Int16 Theta0, Int16 Dtheta, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Use torque test mode.
         Input arguments:
	        MaxTorque:	Maximum test torque value
	        IncrTorque:	Torque increment on each slow sampling period
	        Theta0:		Initial value of electrical angle value
			        Remark: used only for AC motors; set to 0 otherwise
	        Dtheta:		Electric angle increment on each slow sampling period
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetGearingMaster(Boolean Group, Byte SlaveID, Int16 ReferenceBase, Boolean Enable,
                        Boolean SetSlavePos, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Setup master parameters in gearing mode.
         Input arguments:
	        Group
		        TRUE -> set slave group ID with value; 
		        FALSE-> set slave axis ID with SlaveID value;
	        SlaveID:	Axis ID in the case that Group is FALSE or a Group ID when Group is TRUE
	        ReferenceBase:	
		        FROM_MEASURE -> send position feedback
		        FROM_REFERENCE -> send position reference
	        Enable:		TRUE -> enable gearing operation; FALSE -> disable gearing operation
	        SetSlavePos:	TRUE -> initialize slave(s) with master position
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetGearingSlave(Int16 Denominator, Int16 Numerator, Int16 ReferenceBase,
                                           Int16 EnableSlave, Double LimitVariation, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Setup slave parameters in gearing mode.
         Input arguments:
	        Denominator:	Master gear ratio value
	        Numerator:	Slave gear ratio value
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
	        EnableSlave:	
		        SLAVE_NONE -> do not enable slave operation
		        SLAVE_COMMUNICATION_CHANNEL -> enable operation got via a communication channel
		        SLAVE_2ND_ENCODER -> enable operation read from 2nd encoder or P&D inputs
	        LimitVariation:
		        NO_VARIATION (0) -> the external reference is limited at the value set in the Drive Setup
		        A value which can be an acceleration or speed in function of the reference type.
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_MotionSuperposition(Int16 Enable, Int16 Update);
        /*******************************************************************************************
         Function: enable or disable the superposition of the electronic gearing mode with a second 
			        motion mode
         Input arguments:
	        Enable:	if 0, disable the Superposition mode
			        if 1, enable the Superposition mode
	        Update:	if 0, doesn't send UPD command to the drive, in order to take into account the 
			        Superposition mode
			        if 1, sends UPD command to the drive, in order to take into account the 
			        Superposition mode
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetCammingMaster(Boolean Group, Byte SlaveID, Int16 ReferenceBase, Boolean Enable, Int16 MoveMoment);
        /*******************************************************************************************
         Function: Setup master parameters in camming mode.
         Input arguments:
	        Group
		        TRUE -> set slave group ID with (SlaveID + 256) value; 
		        FALSE-> set slave axis ID with SlaveID value;
	        SlaveID:	Axis ID in case Group is FALSE, or group mask otherwise (0 means broadcast)
	        ReferenceBase:	
		        FROM_MEASURE -> send position feedback
		        FROM_REFERENCE -> send position reference
	        Enable:		TRUE -> enable camming operation; FALSE -> disable camming operation
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CamDownload(String pszCamFile, UInt16 wLoadAddress, UInt16 wRunAddress, out UInt16 wNextLoadAddr, out UInt16 wNexRunAddr);
        /*******************************************************************************************
         Function: Download a CAM file to the drive, at a specified address.
         Input arguments:
	        pszCamFile: the name of the file containing the CAM information
	        wLoadAddress: memory address where the CAM is loaded 
	        wRunAddress: memory where the actual CAM table is transfered and executed at run time
         Output arguments:
	        wNextLoadAddr: memory address available for the next CAM file; if 0 there is no memory left
	        wNextRunAddress: memory where the next CAM table is transfered and executed at run time;
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CamInitialization(UInt16 LoadAddress, UInt16 RunAddress);
        /*******************************************************************************************
         Function:	Copies a CAM file from E2ROM to RAM memory. You should not use this if you download CAMs directly to RAM memory 
				        (load address == run address)
         Input arguments:
	        LoadAddress: memory address in E2ROM where the CAM is already loaded
 	        RunAddress: memory address in RAM where the CAM is copied.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetCammingSlaveRelative(UInt16 RunAddress, Int16 ReferenceBase, Int16 EnableSlave, Int16 MoveMoment,
                                                      Int32 OffsetFromMaster, Double MultInputFactor, Double MultOutputFactor);
        /*******************************************************************************************
         Function: Setup slave parameters in relative camming mode.
         Input arguments:
	        RunAddress: memory addresses where the CAM is executed at run time. If any of them is 0 it means that no start address is set
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
	        EnableSlave:	
		        SLAVE_NONE -> do not enable slave operation
		        SLAVE_COMMUNICATION_CHANNEL -> enable operation got via a communication channel
		        SLAVE_2ND_ENCODER -> enable operation read from 2nd encoder or P&D inputs
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        nOffsetFromMaster, nMultInputFactor, nMultOutputFactor: if non-zero, set the correspondent parameter
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetCammingSlaveAbsolute(UInt16 RunAddress, Double LimitVariation, Int16 ReferenceBase, Int16 EnableSlave, Int16 MoveMoment,
                                                      Int32 OffsetFromMaster, Double MultInputFactor, Double MultOutputFactor);
        /*******************************************************************************************
         Function: Setup slave parameters in absolute camming mode.
         Input arguments:
	        RunAddress: memory addresses where the CAM is executed at run time. If any of them is 0 it means that no start address is set
	        LimitVariation:
		        NO_VARIATION (0) -> no limitation on speed value at the value set in the Drive Setup
		        A value which can be an acceleration or speed in function of the reference type.
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
	        EnableSlave:	
		        SLAVE_NONE -> do not enable slave operation
		        SLAVE_COMMUNICATION_CHANNEL -> enable operation got via a communication channel
		        SLAVE_2ND_ENCODER -> enable operation read from 2nd encoder or P&D inputs
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        nOffsetFromMaster, nMultInputFactor, nMultOutputFactor: if non-zero, set the correspondent parameter
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetMasterResolution(Int32 MasterResolution);
        /*******************************************************************************************
         Function: Setup the resolution for the master encoder connected on the second encoder input of the drive.
         Input arguments:
	        MasterResolution: 
		        FULL_RANGE (0) -> select this option if the master position is not cyclic. (e.g. the resolution is equal with the whole 
								        32-bit range of position)
		        Value that reprezents the number of lines of the 2nd master encoder
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SendSynchronization(Int32 Period);
        /*******************************************************************************************
         Function: Setup drives to send synchronization messages.
         Input arguments:
	        Period: the time period between 2 sends
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_Stop();
        /*******************************************************************************************
         Function: Stop the motion.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_QuickStopDecelerationRate(Double Deceleration);
        /*******************************************************************************************
         Function: Set the deceleration rate used for QuickStop or SCurve positioning profile.
         Input Argumernts:
 	        Deceleration: the value of the deceleration rate
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SendPVTPoint(Int32 Position, Double Velocity, UInt32 Time, Int16 PVTCounter);
        /*******************************************************************************************
         Function: Sends a PVT point to the drive.
         Input arguments:
           Position:	drive position for the desired point
           Velocity:	desired velocity of the drive at the point
           Time:			amount of time for the segment
           PVTCounter:	integrity counter for current PVT point
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SendPVTFirstPoint(Int32 Position, Double Velocity, Int32 Time, Int16 PVTCounter,
                                         Int16 PositionType, Int32 InitialPosition, Int16 MoveMoment, Int16 ReferenceBase);
        /*******************************************************************************************
         Function: Sends the first point from a series of PVT points and sets the PVT motion mode.
         Input arguments:
           Position:	drive position for the desired point
           Velocity:	desired velocity of the drive at the point
           Time:			amount of time for the segment
           PVTCounter:	integrity counter for current PVT point
	        PositionType: ABSOLUTE_POSITION or RELATIVE_POSITION
           InitialPosition:	drive initial position at the start of an absolute PVT movement.
							        It is taken into consideration only if an absolute movement is requested
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_PVTSetup(Int16 ClearBuffer, Int16 IntegrityChecking, Int16 ChangePVTCounter,
                                Int16 AbsolutePositionSource, Int16 ChangeLowLevel, Int16 PVTCounterValue, Int16 LowLevelValue);
        /*******************************************************************************************
         Function: For PVT motion mode parametrization and setup.
         Input arguments:
           ClearBuffer:	0 -> nothing
						        1 -> clears the PVT buffer
           IntegrityChecking:	0 -> PVT integrity counter checking is active (default)
								        1 -> PVT integrity counter checking is inactive
           ChangePVTCounter:	0 -> nothing
							        1 -> drive internal PVT integrity counter is changed with the value specified PVTCounterValue
           AbsolutePositionSource:	specifies the source for the initial position in case the PVT motion mode will be absolute
							        0 -> initial position read from PVTPOS0
							        1 -> initial position read from current value of target positio (TPOS)
           ChangeLowLevel:		0 -> nothing
								        1 -> the parameter for BufferLow signaling is changed with the value specified LowLevelValue
           PVTCounterValue:	New value for the drive internal PVT integrity counter
           LowLevelValue:		New value for the level of the BufferLow signal
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SendPTPoint(Int32 Position, Int32 Time, Int16 PTCounter);
        /*******************************************************************************************
         Function: Sends a PT point to the drive.
         Input arguments:
           Position:	drive position for the desired point
           Time:		amount of time for the segment
           PTCounter:	integrity counter for current PT point
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SendPTFirstPoint(Int32 Position, Int32 Time, Int16 PTCounter,
                                         Int16 PositionType, Int32 InitialPosition, Int16 MoveMoment, Int16 ReferenceBase);
        /*******************************************************************************************
         Function: Sends the first point from a series of PT points and sets the PT motion mode.
         Input arguments:
	        Position:	drive position for the desired point
	        Time:		amount of time for the segment
	        PTCounter:	integrity counter for current PT point
	        PositionType: ABSOLUTE_POSITION or RELATIVE_POSITION
	        InitialPosition:	drive initial position at the start of an absolute PT movement.
							        It is taken into consideration only if an absolute movement is requested
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        ReferenceBase:	
		        FROM_MEASURE -> the position reference starts from the actual measured position value
		        FROM_REFERENCE -> the position reference starts from the actual reference position value
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_PTSetup(Int16 ClearBuffer, Int16 IntegrityChecking, Int16 ChangePTCounter,
                                Int16 AbsolutePositionSource, Int16 ChangeLowLevel, Int16 PTCounterValue, Int16 LowLevelValue);
        /*******************************************************************************************
         Function: For PT motion mode parametrization and setup.
         Input arguments:
           ClearBuffer:	0 -> nothing
						        1 -> clears the PT buffer
           IntegrityChecking:	0 -> PT integrity counter checking is active (default)
						        1 -> PT integrity counter checking is inactive
           ChangePVTCounter:	0 -> nothing
						        1 -> drive internal PT integrity counter is changed with the value specified PTCounterValue
           AbsolutePositionSource:	specifies the source for the initial position in case the PT motion mode will be absolute
							        0 -> initial position read from PVTPOS0
							        1 -> initial position read from current value of target positio (TPOS)
           ChangeLowLevel:		0 -> nothing
						        1 -> the parameter for BufferLow signaling is changed with the value specified LowLevelValue
           PTCounterValue:	New value for the drive internal PT integrity counter
           LowLevelValue:		New value for the level of the BufferLow signal
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_MoveSCurveRelative(Int32 RelPosition, Double Speed, Double Acceleration, Int32 JerkTime, Int16 MoveMoment, Int16 DecelerationType);
        /*******************************************************************************************
         Function: For relative S-Curve motion mode.
         Input arguments:
	        RelPosition: 	Relative position reference value
	        Speed: 		Slew speed
	        Acceleration: 	Acceleration  decceleration
	        JerkTime:	The time after the acceleration reaches the desired value.        
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        DecelerationType:	
		        S_CURVE_SPEED_PROFILE -> s-curve speed profile
		        TRAPEZOIDAL_SPEED_PROFILE -> trapezoidal speed profile
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_MoveSCurveAbsolute(Int32 AbsPosition, Double Speed, Double Acceleration, Int32 JerkTime, Int16 MoveMoment, Int16 DecelerationType);
        /*******************************************************************************************
         Function: For absolute S-Curve motion mode.
         Input arguments:
	        AbsPosition: 	Absolute position reference value
	        Speed: 		Slew speed
	        Acceleration: 	Acceleration  decceleration
	        JerkTime:	The time after wich the acceleration reaches the desired value.
	        MoveMoment:		
		        UPDATE_NONE -> setup motion parameters, movement will start latter (on an Update command)
		        UPDATE_IMMEDIATE -> start moving immediate
		        UPDATE_ON_EVENT -> start moving on event
	        DecelerationType:	
		        S_CURVE_SPEED_PROFILE -> s-curve speed profile
		        TRAPEZOIDAL_SPEED_PROFILE -> trapezoidal speed profile
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        /*******************************************************************/
        /*******************EVENT-RELATED functions*************************/
        /*******************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CheckEvent(out Boolean eventDetected);
        /*******************************************************************************************
         Function: Check if the actually active event occured.
         Output arguments:
         eventDetected: TRUE on event detected
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnMotionComplete(Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when the motion is complete.
         Input arguments:
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On motion complete, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnMotorPosition(Int16 PositionType, Int32 Position, Boolean Over, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when motor position is over/under imposed value.
         Input arguments:
	        PositionType: ABSOLUTE_POSITION or RELATIVE_POSITION
	        Position:	Position value to be reached
	        Over:		TRUE -> Look for position over; FALSE -> Look for position below
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnLoadPosition(Int16 PositionType, Int32 Position, Boolean Over, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when load position is over/under imposed value.
         Input arguments:
	        PositionType: ABSOLUTE_POSITION or RELATIVE_POSITION
	        Position:	Position value to be reached
	        Over:		TRUE -> Look for position over; FALSE -> Look for position below
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnMotorSpeed(Double Speed, Boolean Over, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when motor speed is over/under imposed value.
         Input arguments:
	        Speed:		Speed value to be reached
	        Over:		TRUE -> Look for speed over; FALSE -> Look for speed below
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnLoadSpeed(Double Speed, Boolean Over, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when load speed is over/under imposed value.
         Input arguments:
	        Speed:		Speed value to be reached
	        Over:		TRUE -> Look for speed over; FALSE -> Look for speed below
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnTime(UInt16 Time, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event after a time interval.
         Input arguments:
	        Time:		Time after which the event will be set
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnPositionRef(Int32 Position, Boolean Over, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when position reference is over/under imposed value.
         Input arguments:
	        Position:	Position value to be reached
	        Over:		TRUE -> Look for speed over; FALSE -> Look for speed below
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnSpeedRef(Double Speed, Boolean Over, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when speed reference is over/under imposed value.
         Input arguments:
	        Speed:		Speed value to be reached
	        Over:		TRUE -> Look for speed over; FALSE -> Look for speed below
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnTorqueRef(int Torque, Boolean Over, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when torque reference is over/under imposed value.
         Input arguments:
	        Torque:		Torque value to be reached
	        Over:		TRUE -> Look for speed over; FALSE -> Look for speed below
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnEncoderIndex(Int16 IndexType, Int16 TransitionType, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when encoder index is triggered.
         Input arguments:
	        IndexType:	INDEX_1 or INDEX_2
	        TransitionType:	TRANSITION_HIGH_TO_LOW or TRANSITION_LOW_TO_HIGH
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnLimitSwitch(Int16 LSWType, Int16 TransitionType, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when selected limit switch is triggered.
         Input arguments:
	        LSWType:	LSW_NEGATIVE or LSW_POSITIVE
	        TransitionType:	TRANSITION_HIGH_TO_LOW or TRANSITION_LOW_TO_HIGH
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:	TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnDigitalInput(Byte InputPort, Int16 IOState, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when selected input port status is IOState.
         Input arguments:
	        InputPort:	Input port number
	        IOState:	IO_LOW or IO_HIGH
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:		TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetEventOnHomeInput(Int16 IOState, Boolean WaitEvent, Boolean EnableStop);
        /*******************************************************************************************
         Function: Setup event when selected input port status is IOState.
         Input arguments:
	        IOState:	IO_LOW or IO_HIGH
	        WaitEvent:	TRUE -> Wait until event occurs; FALSE -> Continue
	        EnableStop:		TRUE -> On event, stop the motion, FALSE -> Don't stop the motion
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        /*******************************************************************/
        /*******************INPUT / OUTPUT functions************************/
        /*******************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetupInput(Byte nIO);
        /*******************************************************************************************
         Function: Setup IO port as input.
         Input arguments:
	        nIO:	Port number to be set as input
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetInput(Byte nIO, out Byte InValue);
        /*******************************************************************************************
         Function: Get input port status.
         Input arguments:
	        nIO:	Input port number to be read
         Output arguments:
	        InValue:	the input port status value (0 or 1)
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetHomeInput(out Byte InValue);
        /*******************************************************************************************
         Function: Get home input port status.

         Output arguments:
	        InValue:	the input port status value (0 or 1)
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetupOutput(Byte nIO);
        /*******************************************************************************************
         Function: Setup IO port as output.
         Input arguments:
	        nIO:	Port number to be set as output
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetOutput(Byte nIO, Byte OutValue);
        /*******************************************************************************************
         Function: Set output port status.
         Input arguments:
	        nIO:	Output port number to be written
	        OutValue:		Output port status value to be set (0 or 1)
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetMultipleInputs(String pszVarName, out Int16 Status);
        /*******************************************************************************************
         Function:	Read multiple inputs.
         Input arguments:
	        pszVarName: temporary variable name used to read input status
         Output arguments:
	        Status:	value of multiple input status.
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetMultipleOutputs(String pszVarName, Int16 Status);
        /*******************************************************************************************
         Function:	Set multiple outputs (for firmware versions FAxx).
	        pszVarName: temporary variable name used to set output status
	        Status: value to be set
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetMultipleOutputs2(Int16 SelectedPorts, Int16 Status);
        /*******************************************************************************************
         Function:	Set multiple outputs (for firmware versions FBxx).
	        SelectedPorts: port mask. Set bit n to 1 if you want to update the status of port n.
	        Status: value to be set
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        /*******************************************************************/
        /*******************General use********************************/
        /*******************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SendDataToHost(Byte HostAddress, UInt32 StatusRegMask, UInt16 ErrorRegMask);
        /*******************************************************************************************
         Function: Send status and error registers to host.
         Input arguments:
	        HostAddress:	axis ID of host
	        StatusRegMask: bit mask for status register 
	        ErrorRegMask: bit mask for error register
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_OnlineChecksum(UInt16 startAddress, UInt16 endAddress, out UInt16 checksum);
        /*******************************************************************************************
         Function: Get checksum of a memory range. 
		        startAddress:	start memory address
		        endAddress: end memory address
         Output arguments:
		        checksum: checksum (sum modulo 0xFFFF) of a memory range returned by the active drive/motor  
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_DownloadProgram(String pszOutFile, out UInt16 wEntryPoint);
        /*******************************************************************************************
         Function: Download a COFF formatted file to the drive, and return the entry point of that file.
         Input arguments:
	        pszOutFile:	Path to the output TML object file
         Output arguments:
	        wEntryPoint: the entry point (start address) of the downloaded file
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_DownloadSwFile(String pszSwFile);
        /*******************************************************************************************
         Function: Download a .sw file to the drive's E2PROM.
         Input arguments:
	       pszSwFile:	Path to the SW file generated from EasyMotion Studio/EasySetUp
         Output arguments:
	       return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GOTO(UInt16 address);
        /*******************************************************************************************
         Function: Execute a GOTO instruction on the drive.
         Input arguments:
	        address: program memory address of the instruction
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GOTO_Label(String pszLabel);
        /*******************************************************************************************
         Function: Execute a GOTO instruction on the drive.
         Input arguments:
	        pszLabel: label of the instruction
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CALL(UInt16 address);
        /*******************************************************************************************
         Function: Execute a CALL instruction on the drive.
         Input arguments:
	        address: address of the procedure
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CALL_Label(String pszFunctionName);
        /*******************************************************************************************
         Function: Execute a CALL instruction on the drive.
         Input arguments:
	        pszFunctionName: name of the procedure to be executed
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CancelableCALL(UInt16 address);
        /*******************************************************************************************
         Function: Execute a cancelable call (CALLS) instruction on the drive.
         Input arguments:
	        address: address of the procedure
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CancelableCALL_Label(String pszFunctionName);
        /*******************************************************************************************
         Function: Execute a cancelable call (CALLS) instruction on the drive.
         Input arguments:
	        pszFunctionName: name of the procedure to be executed
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_ABORT();
        /*******************************************************************************************
         Function:  Execute ABORT instruction on the drive (aborts execution of a procedure called 
			        with cancelable call instruction).
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_Execute(String pszCommands);
        /*******************************************************************************************
         Function: Execute TML commands entered in TML source code format (as is entered in Command Interpreter).
         Input arguments:
	        pszCommands: String containing the TML source code to be executed. Multiple lines are allowed.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_ExecuteScript(String pszFileName);
        /*******************************************************************************************
         Function: Execute TML commands in TML source code, from a script file (as is entered in Command Interpreter).
         Input arguments:
	        pszFileName: The name of the file containing the TML source code to be executed.
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_GetOutputOfExecute(StringBuilder pszOutput, int nMaxChars);
        /*******************************************************************************************
         Function: Return the TML output code of the last previously executed library function call.
         Input arguments:
	        pszOutput: String containing the TML source code generated at the last library function call.
	        nMaxChars: maximum number of characters to return in the string
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_SetupLogger(UInt16 wLogBufferAddr, UInt16 wLogBufferLen, UInt16[] arrayAddresses, UInt16 countAddr, UInt16 period);
        /*******************************************************************************************
         Function: Setup logger parameters (could be set up on a group/broadcast destination).
         Input arguments:
	        wLogBufferAddr: The address of logger buffer in drive memory, where data will be stored during logging
	        wLogBufferLen: The length in WORDs of the logger buffer
	        arrayAddresses: An array containing the drive memory addresses to be logged
	        countAddr: The number of memory addresses to be logged
	        period: How offen to log the data: a value between 1 and 7FFF (useful only for new generation drives).
		        If it is different than 1, one set of data will be stored at every "period" samplings. 
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_StartLogger(UInt16 wLogBufferAddr, Byte LogType);
        /*******************************************************************************************
         Function: Start the logger on a drive (could be started on a group/broadcast destination).
         Input arguments:
	        wLogBufferAddr: address of logger buffer (previously set by TS_SetupLogger)
	        LogType: 
			        LOGGER_FAST: logging occurs in fast sampling control loop (current loop)
			        LOGGER_SLOW: logging occurs in slow sampling control loop (position/speed loop)
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CheckLoggerStatus(UInt16 wLogBufferAddr, out UInt16 status);
        /*******************************************************************************************
         Function: Check logger status. (destination must be a single axis).
         Input arguments:
	        wLogBufferAddr: address of logger buffer (previously set by TS_SetupLogger)
         Output arguments:
	        status: Number of points still remaining to capture; if it is 0, the logging is completed
	        return:		TRUE if no error; FALSE if error
        ******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_UploadLoggerResults(UInt16 wLogBufferAddr, UInt16[] arrayValues, ref UInt16 countValues);
        /*******************************************************************************************
         Function: Upload logged data from the drive (destination must be a single axis).
         Input arguments:
	        wLogBufferAddr: address of logger buffer (previously set by TS_SetupLogger)
	        arrayValues:	Pointer to the array where the uploaded data is stored on the PC
	        countValues:	The size of arrayValues
         Output arguments:
	        arrayValues:	uploaded logger data
	        countValues:	The size of actualized data (lower or equal with countValues input value)
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern void TS_RegisterHandlerForUnrequestedDriveMessages(pfnCallbackRecvDriveMsg handler);
        /*******************************************************************************************
         Function: Register application's handler for unrequested drive messages.
         Input arguments:
         pfnCallbackRecvDriveMsg:		pointer to handler
         Output arguments:
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_CheckForUnrequestedDriveMessages();
        /*******************************************************************************************
         Function: Check if there are new unrequested drive messages and call handler for every message received.
         Input arguments:
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

        [DllImport("TML_lib.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Boolean TS_DriveInitialisation();
        /*******************************************************************************************
         Function: Execute ENDINIT command and verify if the setup table is valid. This function 
		           must be called only after TS_LoadSetup & TS_SetupAxis & TS_SelectAxis are called.
		           If the setup table is invalid then use the EasySetUp or EasyMotion Studio to download
		           a valid setup table. Please note that after the setup table download, the drive must
		           be reset in order to activate the new setup data.
         Input arguments:
         Output arguments:
	        return:		TRUE if no error; FALSE if error
        *******************************************************************************************/

    }

    public class TMLComm
    {
        public const Int32 LOG_NONE = 0;
        public const Int32 LOG_ERROR = 1;
        public const Int32 LOG_WARNING = 2;
        public const Int32 LOG_TRAFFIC = 3;

        //supported CAN protocols
        public const Byte PROTOCOL_TMLCAN = 0x00;    //use TMLCAN protocol (default, 29-bit identifiers)
        public const Byte PROTOCOL_TECHNOCAN = 0x80; //use TechnoCAN protocol (11-bit identifiers)
        public const Byte PROTOCOL_MASK = 0x80;      //this bits are used for specifying CAN protocol through nChannelType param of MSK_OpenComm function

        /***** supported CAN devices *****************************
        CHANNEL_IXXAT_CAN - see http://www.ixxat.com
        CHANNEL_SYS_TEC_USBCAN - see www.systec-electronic.com
        CHANNEL_ESD_CAN - see http://www.esd-electronics.com
        CHANNEL_PEAK_SYS_PCAN_* - see http://www.peak-system.com
        **********************************************************/
        public const Byte CHANNEL_RS232 = 0;
        public const Byte CHANNEL_RS485 = 1;
        public const Byte CHANNEL_IXXAT_CAN = 2;
        public const Byte CHANNEL_SYS_TEC_USBCAN = 3;
        public const Byte CHANNEL_PEAK_SYS_PCAN_PCI = 4;
        public const Byte CHANNEL_ESD_CAN = 5;
        public const Byte CHANNEL_PEAK_SYS_PCAN_ISA = 6;
        public const Byte CHANNEL_PEAK_SYS_PCAN_PC104 = CHANNEL_PEAK_SYS_PCAN_ISA; //Same with PCAN_ISA
        public const Byte CHANNEL_PEAK_SYS_PCAN_USB = 7;
        public const Byte CHANNEL_PEAK_SYS_PCAN_DONGLE = 8;
        public const Byte CHANNEL_VIRTUAL_SERIAL = 15;
        public const Byte CHANNEL_XPORT_IP = 16;

        //MSK Message structure
        public const UInt16 DSP_PC_FLAG = 0x1;
        public const UInt16 DSP_GROUP_FLAG = 0x1000;
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct MSK_MSG
        {
            public UInt16 m_Len;
            public UInt16 m_Addr;
            public UInt16 m_OpCode;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public UInt16[] m_Data;
        }

        /****************************************************************************
        Callback function used by client application for handling debug messages
        /*****************************************************************************/
        public delegate void MSK_CallbackDebugLog(String pszMsg);

        /****************************************************************************
        Callback function used by client application for handling unsolicited
        messages which this driver receives in unexpected places
        /*****************************************************************************/
        public delegate void MSK_CallbackRecvMsg(ref MSK_MSG pMsg);

        /****************************************************************************
        Callback function used by client application to manage complex 
        MSK functions, like downloading or flash burning/
        /*****************************************************************************/
        public delegate bool MSK_Callback(Int32 nMSGType, String pcszMsg,
                                        Single nElpsTime, String pUsrData);
        public const Int32 MSK_CALLBACK_APPEND_MSG = 1;
        public const Int32 MSK_CALLBACK_YESNO_MSG = 2;
        public const Int32 MSG_CALLBACK_REPLACE_LASTLINE = 3;
        public const Int32 MSG_CALLBACK_APPEND_NEWLINE_MSG = 4;
        /*	Parameters:
	        nMSGType:	message type;
	        pcszMsg:	message to be displayed by client application;
				        the meaning of this message depends by the "nMSGType" parameter:
					        "MSK_CALLBACK_APPEND_MSG" - append the message to the 
						        client application output;
					        "MSK_CALLBACK_YESNO_MSG" - prompts user for a question;
					        "MSG_CALLBACK_REPLACE_LASTLINE" - replace last line of the
						        client application output with the provided message;
					        "MSG_CALLBACK_APPEND_NEWLINE_MSG" - append the message at
						        the beginning of a new line;
	        nElpsTime:	relative elapsed time (percent value);
	        pUsrData:	pointer to user data provided by client application;
				        usually used to identify the output device.
	        Return value:
				        TRUE:	continue to execute MSK function;
				        FALSE:	exit from MSK function with user abort error code; */

        /* DSP memory space */
        public const Byte MSK_PM = 0;	/*Program memory space */
        public const Byte MSK_DM = 1;	/*Data memory space */
        public const Byte MSK_SPI = 2;	/*SPI memory space */

        /*****************************************************************************/
        /* All MSK exported function use the same meaning of the return value:
	        TRUE:	if the function is successful;
	        FALSE:	error; Use MSK_GetLastError for more details*/
        /*****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern String MSK_GetLastError();

        /*****************************************************************************
           MSK_RegisterDebugLogHandler: set destination of debugging messages and level of debugging
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern void MSK_RegisterDebugLogHandler(Int32 nLevel, MSK_CallbackDebugLog pfnCallback);
        /*****************************************************************************
           Parameters:
           pfnCallback - pointer to callback function who receive debug messages
           nLevel - level of logging (1 = error, 2 = error and warning, 3 = traffic, error and warning)
           if pfnCallback == NULL or nLevel <=0, logging is disabled
        *****************************************************************************/

        /****************************************************************************
        MSK_RegisterReceiveUnsolicitedMsgHandler: 
	        register a callback called when unexpected messages are received instead acknowledge
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern void MSK_RegisterReceiveUnsolicitedMsgHandler(MSK_CallbackRecvMsg pfnCallback);
        /* 
	        pfnCallback: pointer to the callback function
        */

        /*****************************************************************************
           MSK_OpenComm: function that opens a communication channel
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Int32 MSK_OpenComm(String pszComm, UInt16 nHostAddress, bool bExclusive, Byte nChannelType);
        /*****************************************************************************
           Parameters:
              pszComm:   identifier for the communication port
			        RS232 and RS485:
					        "1","2","3"... -> COM1, COM2, COM3...
			        CHANNEL_IXXAT_CAN: "1" .. "4"
			        CHANNEL_SYS_TEC_USBCAN and CHANNEL_ESD_CAN: "0" .. "10"
			        CHANNEL_PEAK_SYS_PCAN_PCI: "1" or "2"
			        CHANNEL_XPORT_IP: "IP" or "hostname"
              nHostAddress: host board address
              nChannelType: Type of communcation (CHANNEL_* defines) with an optional protocol mask (PROTOCOL_* defines)
		        bExclusive:	TRUE value for exclusive using.
           Return:
              -1 means error
              otherwise is the port file descriptor. 
        *****************************************************************************/

        /*****************************************************************************
           MSK_SelectComm: function that select a communication channel.
              Note: MSK_OpenComm automatically selects the communication port. If you work only
                 with one communication port there is no need to call this function
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Int32 MSK_SelectComm(Int32 fdComm);
        /*****************************************************************************
           Parameters:
              fdComm: the communication port file descriptor.
           Return:
              if error it returns -1
		        else returns previous selected fdComm
        *****************************************************************************/

        /*****************************************************************************
           MSK_CloseComm: function that closes the communication channel 
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_CloseComm(Int32 fdComm);
        /*****************************************************************************
           Parameters:
              fdComm: the communication port file descriptor. -1 means current selected.
        *****************************************************************************/

        /*****************************************************************************/
        /*	MSK_UpdateSettings: function that reloads settings from  Windows registry  path HKEY_CURRENT_USER\Software\TechnoSoft\TMLCOMM*/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_UpdateSettings();

        /*****************************************************************************
           MSK_SetBaudRate: set device baud rate.
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_SetBaudRate(UInt32 nBaud);
        /*****************************************************************************
	        Parameters:
	        nBaud:		new baud rate (serial lines have baud rates of 9600, 19200, 38400, 57600 or 115200)
        *****************************************************************************/

        /*****************************************************************************
           MSK_GetBaudRate: get device baud rate.
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern UInt32 MSK_GetBaudRate();
        /*****************************************************************************
           Return: 0 on error or baudrate on success
        *****************************************************************************/

        /*****************************************************************************
	        MSK_GetBytesCountInQueue: Test if any character is available on the communication buffer device 
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern Int32 MSK_GetBytesCountInQueue();
        /*****************************************************************************
	        Returns:
	        if < 0		erorr
	        else number of bytes in serial queue
        *****************************************************************************/

        /*****************************************************************************
           MSK_SetActiveDrive: set active drive (messages are sent/received to/from this axis)
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_SetActiveDrive(UInt16 AxisID);
        /*****************************************************************************
           Parameters:
              AxisID: the address of the drive.
        *****************************************************************************/

        /*****************************************************************************
           MSK_GetActiveDrive: retrieve the current active drive
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_GetActiveDrive(out UInt16 pAxisID);
        /*****************************************************************************
           Parameters:
              pAxisID: pointer to a buffer which will contain the address of the drive.
        *****************************************************************************/

        /*****************************************************************************
           MSK_SetHostDrive: Set the address of the host drive
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_SetHostDrive(UInt16 AxisID);
        /*****************************************************************************
           Parameters:
              AxisID: the address of the drive.
        *****************************************************************************/

        /*****************************************************************************
           MSK_GetHostDrive: Get the address of the host drive
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_GetHostDrive(out UInt16 pAxisID);
        /*****************************************************************************
           Parameters:
              pAxisID: pointer to a buffer which will contain the address of the drive.
        *****************************************************************************/

        /*****************************************************************************
           MSK_GetDriveVersion: retrieve firmware version of the active drive
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_GetDriveVersion(StringBuilder szDriveVersion);
        /*****************************************************************************
           Parameters:
              szDriveVersion: pointer to a buffer of 5 characters (not including null terminator)
                 which will contain the firmware version.
        *****************************************************************************/

        /*****************************************************************************
           MSK_ResetDrive: reset the active drive.
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_ResetDrive();

        /*****************************************************************************
           MSK_SendMessage: send a message. 
		        Note: The active drive has no effect on this function.
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_SendMessage(ref MSK_MSG pMsg);
        /*****************************************************************************
           Parameters:
              pMsg: pointer to a message structure which will be sent
           CAUTION: pMsg->wAddr must be equal with (axisID << 4) or ((1 << (4+groupID-1)) | DSP_GROUP_FLAG)
        *****************************************************************************/

        /*****************************************************************************
           MSK_ReceiveMessage: receive a message. The active drive has no effect on this function.
		        Note: The active drive has no effect on this function.
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_ReceiveMessage(out MSK_MSG pMsg);
        /*****************************************************************************
           Parameters:
              pMsg: pointer to a message structure which will contain the received message
        *****************************************************************************/

        /*****************************************************************************
	        MSK_CheckSum: Ask for checksum of active drive's memory.
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_CheckSum(UInt16 nStartAddr, UInt16 nEndAddr, out UInt16 pCheckSum);
        /*****************************************************************************
           Parameters:
		        nStartAddr:	start memory address
		        nEndAddr: end memory address
		        pCheckSum: received CheckSum from the drive
        *****************************************************************************/

        /*****************************************************************************
           MSK_SendData: This function transfers a block of data into active drive's memory 
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_SendData(UInt16[] pData, UInt16 nAddr, Byte memType, UInt16 nLength);
        /*****************************************************************************
           Parameters:
                pData:      pointer to data buffer;
                nAddr:      memory address where the data will be stored
                memType:    memory space where the data will be stored (MSK_DM, MSK_PM OR MSK_SPI)
                nLength:    number of words to be stored
        *****************************************************************************/

        /*****************************************************************************
           MSK_ReceiveData: This function read a block of data from active drive's memory 
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_ReceiveData(UInt16[] pData, UInt16 nAddr, Byte memType, UInt16 nLength);
        /*****************************************************************************
           Parameters:
              pData:      pointer to data buffer;
              nAddr:      memory address from where the data will be readed
              memType:      memory space from where the data will be readed (MSK_DM, MSK_PM OR MSK_SPI)
              nLength:      number of words to be readed
        *****************************************************************************/

        /*****************************************************************************
           MSK_SendBigData: This function transfers a block of data into active drive's memory, 
		        Note: the only difference from MSK_SendData is support for completion measurement
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_SendBigData(UInt16[] pData, UInt16 nAddr, Byte memType, UInt16 nLength, MSK_Callback pf, String pUsrData);
        /*****************************************************************************
           Parameters:
              pData:      pointer to data buffer;
              nAddr:      memory address where the data will be stored
              memType:    memory space where the data will be stored (MSK_DM, MSK_PM OR MSK_SPI)
              nLength:    number of words to be stored
              pf:         pointer to a callback function used to display progress status of this operation
              pUsrData:   pointer to user data passed to callback function
        *****************************************************************************/

        /*****************************************************************************
           MSK_ReceiveBigData: This function read a block of data from active drive's memory. 
		        Note: the only difference from MSK_ReceiveData is support for completion measurement
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_ReceiveBigData(UInt16[] pData, UInt16 nAddr, Byte memType, UInt16 nLength, MSK_Callback pf, String pUsrData);
        /*****************************************************************************
           Parameters:
              pData:      pointer to data buffer;
              nAddr:      memory address from where the data will be readed
              memType:    memory space from where the data will be readed (MSK_DM, MSK_PM OR MSK_SPI)
              nLength:    number of words to be read
              pf:         pointer to a callback function used to display progress status of this operation
              pUsrData:   pointer to user data passed to callback function
        *****************************************************************************/

        /*****************************************************************************
           MSK_COFFDownload: download a COFF formatted file into drive's memory (PM or SPI)
        *****************************************************************************/
        [DllImport("TMLcomm.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        public static extern bool MSK_COFFDownload(String file_name, out UInt16 pEntry, out UInt16 pBeginAddr, out UInt16 pEndAddr,
        MSK_Callback pf, String pUsrData, UInt16 wSPISize);
        /*****************************************************************************
           Parameters:
              file_name:            name of the COFF file
              pEntry(out param):   will contain the entry point of the downloaded OUT (can be NULL)
              pBeginAddr(out param):   will contain the smallest location address written by download (can be NULL)
              pEndAddr(out param):   will contain the biggest location address written by download (can be NULL)
              pf:                  pointer to a callback function used to control the download process (can be NULL)
              pUsrData:            user data passed to callback function (can be NULL)
              wSPISize:            size (in WORDS) of the SPI memory. Used only for testing if a section is out of memory.
                                      The range is between 0 and 0x4000.
        *****************************************************************************/
    }
}
