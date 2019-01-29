# TechnosoftMotorsControl

**TechnosoftMotorsCotnrol** is a wrapper for **TML_lib.dll** file enabling easy control of 3 axis machines.
current functionality is very limited due to a specific use case, see [Contribution](#contribution) section to read more about making changes.

## Getting started
Before you start you need make sure that all motors are configured correctly and flashed with working firmware, use [EasySetup](https://www.technosoftmotion.com/en/easysetup-free-download) to do so, additionally you will need to export a configuration file for each motor, open each setup file and then: Setup > 'Export to TML_LIB...' and save the files in known location.

Now clone the project and open it with visual studio:
`git clone https://github.com/20lives/TechnosoftMotorsControl.git`

Open the file: `TechnosoftMotorsControl.cs` and update the configuration variables:

Update the port used and boudrate:
```
CHANNEL_NAME = "COM8";
BAUDRATE = 115200;
```

Replace these with the exported files saved before:
```
x_file = @"C:\MOTORS_TML\X1.t.zip";
y_file = @"C:\MOTORS_TML\Y2.t.zip";
pz_file = @"C:\MOTORS_TML\Z3.t.zip";
```

Update the used axis id for each motor:
```
x_id = 1;
y_id = 2;
x_id = 1;
```

Select the id of the motor that is directly connected to the computer:
```
host_id = 1;
```

## Methods

**bool HomeAxes()**

**void SetSpeed(double speed)**

**void SetAcceleration(double acceleration)**

**bool MoveAbsAsync(char axis, int/double position)**

**bool MoveAbs(char axis, int/double position)**

**bool MoveRelAsync(char axis, int/double position)**

**bool MoveRel(char axis, int/double position)**

**bool WaitForMotionComplete(char axis) / WaitForMotionComplete()**

## Contribution
If you desire for code changes updates please assist [TML_LIB User Manual](http://media.oem.se/Archive/FilesArchive/29979.pdf)
