using System;
using System.Collections.Generic;
using TML;

namespace TechnsoftMotorsControl
{
    public class MotorsControl
    {
        public struct Motor
        {
            public byte id;
            public string file;
            public double micron_to_rot;
            public int limit;
        };
        private static Mutex mutex = new Mutex();

        public IDictionary<char, Motor> motors;

        private double speed = 10;
        private double acceleration = 0.3;

        private int channel_id = -1;
        private byte host_id = 0;
        private string channel_name = "";

        private const uint BAUDRATE = 115200;
        private const byte CHANNEL_TYPE = TMLLib.CHANNEL_RS232;

        public const bool NO_STOP = false;
        public const bool NO_ADDITIVE = false;
        public const bool WAIT_EVENT = true;

        public MotorsControl()
        {
          motors = new Dictionary<char, Motor>();
        }

        public bool Init()
        {
            if (channel_name == "" || host_id == 0)
            {
                return false;
            }
            if (motors.Count <= 0)
            {
                return false;
            }
            mutex.WaitOne();
            if (!InitCommunicationChannel())
            {
                return false;
            }
            if (!InitAxes())
            {
                return false;
            }
            return true;
            mutex.ReleaseMutex();
        }


        public void SetChannelName(string name)
        {
            channel_name = name;
        }

        public void SetHostId(byte id)
        {
            host_id = id;
        }

        public void AddMotor(char symbol, byte id, string file, double micron_to_rot, int limit)
        {
            Motor motor;
            motor.id = id;
            motor.file = file;
            motor.micron_to_rot = micron_to_rot;
            motor.limit = limit;
            motors.Add(symbol, motor);
        }

        public bool InitCommunicationChannel()
        {
            channel_id = TMLLib.TS_OpenChannel(CHANNEL_NAME, CHANNEL_TYPE, host_id, BAUDRATE);
            return (channel_id < 0) ? false : true;
        }

        private bool SelectChannel()
        {
            return TMLLib.TS_SelectChannel(channel_id);
        }

        public bool InitAxes()
        {
            int idxSetup = -1;
            foreach (var motor in motors.Values)
            {
                idxSetup = InitAxis(motor.id, motor.file);
                if (idxSetup == -1)
                {
                    return false;
                }
            }
            if (motors.Count > 1)
            {
                if (!TMLLib.TS_SetupBroadcast(idxSetup)) return false;

                if (!TMLLib.TS_SelectBroadcast()) return false;
            }

            if (!TMLLib.TS_Power(TMLLib.POWER_ON)) return false;

            foreach (var motor in motors.Values)
            {
                UInt16 sAxiOn_flag = 0;
                while (sAxiOn_flag == 0)
                {
                    if (!TMLLib.TS_SelectAxis(motor.id)) return false;
                    if (!TMLLib.TS_ReadStatus(TMLLib.REG_SRL, out sAxiOn_flag)) return false;
                    sAxiOn_flag = (UInt16)((sAxiOn_flag & 1 << 15) != 0 ? 1 : 0);
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
            mutex.WaitOne();
            if (!SelectChannel()) return -1;
            foreach (var motor in motors.Values)
            {
                if (!HomeAxis(motor.id))
                {
                    return motor.id;
                }
            }
            mutex.ReleaseMutex();
            return 0;
        }

        public Dictionary<char, double> GetSystemPosition()
        {
            Dictionary<char, double> pos = new Dictionary<char, double>();
            foreach (var motor in motors.Keys)
            {
                pos.Add(motor, GetPosition(motor));
            }
            return pos;
        }

        public bool MoveAbsAsync(char axis, int position)
        {
            if (position > motors[axis].limit || position < 0) return false;
            int rot = MicronToRotation(axis, position);
            mutex.WaitOne();
            if (!SelectChannel()) return false;
            if (!TMLLib.TS_SelectAxis(motors[axis].id)) return false;
            if (!TMLLib.TS_MoveAbsolute(rot, speed, acceleration, TMLLib.UPDATE_IMMEDIATE, TMLLib.FROM_REFERENCE)) return false;
            return true;
            mutex.ReleaseMutex();
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
            mutex.WaitOne();
            double final_pos = GetPosition(axis) + (double)position;
            if (final_pos > motors[axis].limit || final_pos < 0) return false;
            int rot = MicronToRotation(axis, position);
            if (!SelectChannel()) return false;
            if (!TMLLib.TS_SelectAxis(motors[axis].id)) return false;
            if (!TMLLib.TS_MoveRelative(rot, speed, acceleration, NO_ADDITIVE, TMLLib.UPDATE_IMMEDIATE, TMLLib.FROM_REFERENCE)) return false;
            mutex.ReleaseMutex();
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
            if (motors.ContainsKey(axis))
            {
                return motors[axis].micron_to_rot;
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
            if (!SelectChannel()) return false;
            if (!TMLLib.TS_SelectAxis(motors[axis].id)) return false;
            if (!TMLLib.TS_SetEventOnMotionComplete(WAIT_EVENT, NO_STOP)) return false;
            return true;
        }

        public bool WaitForMotionComplete()
        {
            mutex.WaitOne();
            foreach (var motor in motors.Keys)
            {
                if (!WaitForMotionComplete(motor)) return false;
            }
            return true;
            mutex.ReleaseMutex();
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
            mutex.WaitOne();
            Int32 position = 0;
            if (!SelectChannel()) return -1;
            if (!TMLLib.TS_SelectAxis(motors[axis].id)) return -1;
            if (!TMLLib.TS_GetLongVariable("APOS", out position))
            {
                return -1;
            }
            mutex.ReleaseMutex();
            return RotationToMicron(axis, position);
        }

        public bool StopMotion(char axis)
        {
            mutex.WaitOne();
            if (!SelectChannel()) return false;
            if (!TMLLib.TS_SelectAxis(motors[axis].id)) return false;
            bool res = TMLLib.TS_Stop();
            mutex.ReleaseMutex();
            return res;
        }

        public bool StopMotion()
        {
            foreach (var motor in motors.Keys)
            {
                if (!StopMotion(motor)) return false;
            }
            return true;
        }
    }
}
