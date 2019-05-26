# TechnosoftMotorsControl

**TechnosoftMotorsCotnrol** is a wrapper for **TML_lib.dll** file enabling easy control of multi axis axis machines.
current functionality is very limited due to a specific use case, see [Contribution](#contribution) section to read more about making changes.

## Getting started
Before you start you need make sure that all motors are configured correctly and flashed with working firmware, use [EasySetup](https://www.technosoftmotion.com/en/easysetup-free-download) to do so, additionally you will need to export a configuration file for each motor, open each setup file and then: Setup > 'Export to TML_LIB...' and save the files in known location.

Now clone the project and reference to it with your visual studio project:
`git clone https://github.com/20lives/TechnosoftMotorsControl.git`
Initialize it as followed:
```
MotorsControl motors = new MotorsControl();
motors.SetChannelName("COM8"); // COM Port used for connecting the host motor
motors.SetHostId(1); // ID of the host motor configured by EasySetup
motors.AddMotor('X', 1, @"C:\MOTORS_TML\X.t.zip", 1.143234, 30000);
motors.AddMotor('Y', 1, @"C:\MOTORS_TML\Y.t.zip", 1.143234, 40000);
motors.AddMotor('Z', 1, @"C:\MOTORS_TML\Z.t.zip", 1, 20000);
motors.Init(); // Initialize connection with added motors
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
