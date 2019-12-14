/* ArduinoConnector by Alan Zucconi
 * http://www.alanzucconi.com/?p=2979
 */
using UnityEngine;
using System;
using System.Collections;
using System.IO.Ports;

namespace KartGame.KartSystems
{
    public class ArduinoConnector : MonoBehaviour
    {

        /* The serial port where the Arduino is connected. */
        [Tooltip("The  serial port where the Arduino is connected")]
        public string port = "COM4";
        /* The baudrate of the serial port. */
        [Tooltip("The baudrate of the serial port")]
        public int baudrate = 9600;

        public SerialPort stream;

        public void Open()
        {
            // Opens the serial port
            stream = new SerialPort(port, baudrate);
            // Debug.Log("Stream port name is:");
            // Debug.Log(stream.PortName);
            stream.ReadTimeout = 50;
            stream.Open();
            // Debug.Log("Connector Open");
            //this.stream.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
        }

        public void WriteToArduino(string message)
        {
            // Send the request
            if (message != null)
            {
                stream.WriteLine(message);
                stream.BaseStream.Flush();
            }
        }

        public string ReadFromArduino(int timeout) // int timeout = 0
        {
            timeout = 80;
            stream.ReadTimeout = timeout;
            try
            {
                return stream.ReadLine();
            }
            catch (TimeoutException)
            {
                return null;
            }
        }


        public IEnumerator AsynchronousReadFromArduino(Action<string> callback, Action fail = null, float timeout = float.PositiveInfinity)
        {
            DateTime initialTime = DateTime.Now;
            DateTime nowTime;
            TimeSpan diff = default(TimeSpan);

            string dataString = null;

            do
            {
                // A single read attempt
                try
                {
                    dataString = stream.ReadLine();
                }
                catch (TimeoutException)
                {
                    dataString = null;
                }

                if (dataString != null)
                {
                    callback(dataString);
                    yield return null;
                }
                else
                    yield return new WaitForSeconds(0.05f);

                nowTime = DateTime.Now;
                diff = nowTime - initialTime;

            } while (diff.Milliseconds < timeout);

            if (fail != null)
                fail();
            yield return null;
        }

        public void Close()
        {
            stream.Close();
            // Debug.Log("Stopping connector.");
            
        }
    }
}
