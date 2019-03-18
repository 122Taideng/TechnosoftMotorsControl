using System;
using Shared;
using TML;

namespace TechnsoftMotorsControl
{
  public class MotorsControl
  {
    private double speed;
    private double acceleration;

    private const string CHANNEL_NAME = "COM8";
    private const uint BAUDRATE = 115200;
    private const byte CHANNEL_TYPE = TMLLib.CHANNEL_RS232;

    public const bool NO_STOP = false;
    public const bool NO_ADDITIVE = false;
    public const bool WAIT_EVENT = true;

    private byte x_id = 1;
    private string x_file = @"C:\MOTORS_TML\X1.t.zip";
    private double x_micron_to_rot = 1.4648;
    private int x_limit = 500000;
    private byte y_id = 2;
    private string y_file = @"C:\MOTORS_TML\Y2.t.zip";
    private double y_micron_to_rot = 1.4648;
    private int y_limit = 600000;
    private byte z_id = 3;
    private string z_file = @"C:\MOTORS_TML\Z3.t.zip";
    private double z_micron_to_rot = 2.4446;
    private int z_limit = 50000;

    private byte host_id = 3;

    const char X = 'X';
    const char Y = 'Y';
    const char Z = 'Z';

    public MotorsControl()
    {
      speed = 10;
      acceleration = 0.3;
      if (!InitCommunicationChannel())
      {
        return;
      }
      if (!InitAxes())
      {
        return;
      }
    }

    public bool InitCommunicationChannel()
    {
      return (TMLLib.TS_OpenChannel(CHANNEL_NAME, CHANNEL_TYPE, host_id, BAUDRATE) < 0) ? false : true;
    }

    public bool InitAxes()
    {
      UInt16 sAxiOn_flag_01 = 0;
      UInt16 sAxiOn_flag_02 = 0;
      UInt16 sAxiOn_flag_03 = 0;

      if (InitAxis(x_id, x_file) == -1)
        return false;
      if (InitAxis(y_id, y_file) == -1)
        return false;
      int idxSetup = InitAxis(z_id, z_file);
      if (idxSetup == -1)
        return false;

      if (!TMLLib.TS_SetupBroadcast(idxSetup))
        return false;

      if (!TMLLib.TS_SelectBroadcast())
        return false;

      if (!TMLLib.TS_Power(TMLLib.POWER_ON))
        return false;

      while ((sAxiOn_flag_01 == 0) && (sAxiOn_flag_02 == 0) && (sAxiOn_flag_03 == 0))
      {
        if (sAxiOn_flag_01 == 0)
        {
          if (!TMLLib.TS_SelectAxis(x_id))
            return false;
          if (!TMLLib.TS_ReadStatus(TMLLib.REG_SRL, out sAxiOn_flag_01))
            return false;
          sAxiOn_flag_01 = (UInt16)((sAxiOn_flag_01 & 1 << 15) != 0 ? 1 : 0);
        }

        if (sAxiOn_flag_02 == 0)
        {
          if (!TMLLib.TS_SelectAxis(y_id))
            return false;
          if (!TMLLib.TS_ReadStatus(TMLLib.REG_SRL, out sAxiOn_flag_02))
            return false;
          sAxiOn_flag_02 = (UInt16)((sAxiOn_flag_02 & 1 << 15) != 0 ? 1 : 0);
        }

        if (sAxiOn_flag_03 == 0)
        {
          if (!TMLLib.TS_SelectAxis(z_id))
            return false;
          if (!TMLLib.TS_ReadStatus(TMLLib.REG_SRL, out sAxiOn_flag_03))
            return false;
          sAxiOn_flag_02 = (UInt16)((sAxiOn_flag_03 & 1 << 15) != 0 ? 1 : 0);
        }
      }
      return true;
    }

    private int InitAxis(byte axis_id, string axis_file)
    {
      int idxSetup = TMLLib.TS_LoadSetup(axis_file);
      if (idxSetup < 0)
        return -1;

      if (!TMLLib.TS_SetupAxis(axis_id, idxSetup))
        return -1;

      if (!TMLLib.TS_SelectAxis(axis_id))
        return -1;

      if (!TMLLib.TS_DriveInitialisation())
        return -1;
      return idxSetup;
    }

    private bool HomeAxis(byte axis_id)
    {
      Int32 position = -100000000;
      Int32 home_position = 0;
      Int32 back_step = 1000;
      double high_speed = 20;
      double low_speed = 1.0;
      double acceleration = 0.6;

      if (!TMLLib.TS_SelectAxis(axis_id))
        return false;
      if (!TMLLib.TS_MoveRelative(position, high_speed, acceleration, NO_ADDITIVE, TMLLib.UPDATE_IMMEDIATE, TMLLib.FROM_REFERENCE))
        return false;
      if (!TMLLib.TS_SetEventOnLimitSwitch(TMLLib.LSW_NEGATIVE, TMLLib.TRANSITION_LOW_TO_HIGH, WAIT_EVENT, NO_STOP))
        return false;
      if (!TMLLib.TS_SetEventOnMotionComplete(WAIT_EVENT, NO_STOP))
        return false;
      if (!TMLLib.TS_MoveRelative(back_step, low_speed, acceleration, NO_ADDITIVE, TMLLib.UPDATE_IMMEDIATE, TMLLib.FROM_REFERENCE))
        return false;
      if (!TMLLib.TS_SetEventOnMotionComplete(WAIT_EVENT, NO_STOP))
        return false;
      if (!TMLLib.TS_SetPosition(home_position))
        return false;
      return true;
    }

    public bool HomeAxes()
    {
      if (!HomeAxis(z_id) || !HomeAxis(y_id) || !HomeAxis(x_id))
      {
        return false;
      }
      return true;
    }

    private byte CharToId(char axis)
    {
      switch (axis)
      {
        case X:
          return x_id;
        case Y:
          return y_id;
        case Z:
          return z_id;
      }
      return 0;
    }

    public Point3D GetSystemPosition()
    {
      Point3D pos;
      pos.x = GetPosition('X');
      pos.y = GetPosition('Y');
      pos.z = GetPosition('Z');
      return pos;
    }

    public bool MoveAbsAsync(char axis, int position)
    {
      if (position > GetLimit(axis) || position < 0)
        return false;
      byte axis_id = CharToId(axis);
      int rot = MicronToRotation(axis, position);
      if (!TMLLib.TS_SelectAxis(axis_id))
        return false;
      if (!TMLLib.TS_MoveAbsolute(rot, speed, acceleration, TMLLib.UPDATE_IMMEDIATE, TMLLib.FROM_REFERENCE))
        return false;
      return true;
    }

    public bool MoveAbsAsync(char axis, double position)
    {
      int int_position = (int)Math.Round((double)position);
      if (!MoveAbsAsync(axis, int_position))
        return false;
      return true;
    }

    public bool MoveAbs(char axis, int position)
    {
      if (!MoveAbsAsync(axis, position))
        return false;
      if (!WaitForMotionComplete(axis))
        return false;
      return true;
    }

    public bool MoveAbs(char axis, double position)
    {
      int int_position = (int)Math.Round((double)position);
      return MoveAbs(axis, int_position);
    }

    public bool MoveRelAsync(char axis, int position)
    {
      double final_pos = GetPosition(axis) + (double)position;
      if (final_pos > GetLimit(axis) || final_pos < 0)
        return false;
      byte axis_id = CharToId(axis);
      int rot = MicronToRotation(axis, position);
      if (!TMLLib.TS_SelectAxis(axis_id))
        return false;
      if (!TMLLib.TS_MoveRelative(rot, speed, acceleration, NO_ADDITIVE, TMLLib.UPDATE_IMMEDIATE, TMLLib.FROM_REFERENCE))
        return false;
      return true;
    }

    public bool MoveRelAsync(char axis, double position)
    {
      int int_position = (int)Math.Round((double)position);
      if (!MoveRelAsync(axis, int_position))
        return false;
      return true;
    }

    public bool MoveRel(char axis, int position)
    {
      if (!MoveRelAsync(axis, position))
        return false;
      if (!WaitForMotionComplete(axis))
        return false;
      return true;
    }

    public bool MoveRel(char axis, double position)
    {
      int int_position = (int)Math.Round((double)position);
      return MoveRel(axis, int_position);
    }

    private double AxisMicronToRot(char axis)
    {
      switch (axis)
      {
        case X:
          return x_micron_to_rot;
        case Y:
          return y_micron_to_rot;
        case Z:
          return z_micron_to_rot;
      }
      return -1;
    }

    private int MicronToRotation(char axis, int micron)
    {
      return (int)Math.Round(micron / AxisMicronToRot(axis));
    }

    private int RotationToMicron(char axis, int rotation)
    {
      return (int)Math.Round(rotation * AxisMicronToRot(axis));
    }

    public bool WaitForMotionComplete(char axis)
    {
      byte axis_id = CharToId(axis);
      if (!TMLLib.TS_SelectAxis(axis_id))
        return false;
      if (!TMLLib.TS_SetEventOnMotionComplete(WAIT_EVENT, NO_STOP))
        return false;
      return true;
    }

    public bool WaitForMotionComplete()
    {
      if (!WaitForMotionComplete(X))
        return false;
      if (!WaitForMotionComplete(Y))
        return false;
      if (!WaitForMotionComplete(Z))
        return false;
      return true;
    }

    public void SetSpeed(double _speed)
    {
      speed = _speed;
    }

    public void SetAcceleration(double _acceleration)
    {
      acceleration = _acceleration;
    }

    public int GetLimit(char axis)
    {
      switch (axis)
      {
        case X:
          return x_limit;
        case Y:
          return y_limit;
        case Z:
          return z_limit;
      }
      return -1;
    }

    public double GetPosition(char axis)
    {
      Int32 position = 0;
      byte axis_id = CharToId(axis);
      if (!TMLLib.TS_SelectAxis(axis_id))
        return -1;
      if (!TMLLib.TS_GetLongVariable("APOS", out position))
      {
        return -1;
      }
      return RotationToMicron(axis, position);
    }
  }
}
