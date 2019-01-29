using System;
using TML;

namespace TechnsoftMotorsControl
{
    public class MotorsControl
    {
        private const string CHANNEL_NAME = "COM8";
        private const uint BAUDRATE = 115200;
        private const byte CHANNEL_TYPE = TMLLib.CHANNEL_RS232;

        private byte host_id = 3;

        public bool InitCommunicationChannel()
        {
            return (TMLLib.TS_OpenChannel(CHANNEL_NAME, CHANNEL_TYPE, host_id, BAUDRATE) < 0) ? false : true;
        }
    }
}