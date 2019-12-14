/*
 * Communicates with the Arduino
 */
using UnityEngine;
using System;
using System.IO.Ports;
using System.Collections;
using System.Collections.Generic;
using KartGame.Track;
using KartGame.KartSystems;
using Object = UnityEngine.Object;

namespace KartGame.KartSystems
{
    public class ArduinoCommunication : MonoBehaviour
{
        /* The serial port where the Arduino is connected. */
        [Tooltip("The serial port where the Arduino is connected")]
        public string port = "COM4";
        /* The baudrate of the serial port. */
        [Tooltip("The baudrate of the serial port")]
        public int baudrate = 9600;

        private SerialPort stream;

        public void Open()
        {
            // Opens the serial port
            stream = new SerialPort(port, baudrate);
            stream.ReadTimeout = 50;
            stream.Open();
            //this.stream.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
        }
        
        public void WriteToArduino(string message)
        {
            // Send the request
            stream.WriteLine(message);
            stream.BaseStream.Flush();
        }

        public string ReadFromArduino(int timeout = 0)
        {
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
        }


        [Tooltip("A reference to a script that provides information about the kart's movement, usually the KartMovmeent script.  This must implement IKartInfo.")]
        [RequireInterface(typeof(IKartInfo))]
        public Object kartMovement;

        IKartInfo m_KartMovement;

        // Start is called before the first frame update
        void Start()
        {
            Debug.Log("ArduinoComm has started");

            


        }

        void Awake()
        {
            Debug.Log("ArduinoComm is awake.");
            m_KartMovement = kartMovement as IKartInfo;
        }

        // Update is called once per frame
        void Update()
        {
            Debug.Log("ArduinoComm is updating.");
            // get the latest kart movement stats
            // IKartInfo kartInfo = m_KartMovement.GetKartInfo();

            
            if (m_KartMovement == null)
            {
                Debug.Log("kartInfo is null");
                return;
            }

            // get the kart speed information
            float speed = m_KartMovement.LocalSpeed;
            string speedstr = speed.ToString();

            // send car speed to stream to Arduino
            WriteToArduino(speedstr);

            // Have Unity read from Arduino
            StartCoroutine(
                AsynchronousReadFromArduino
                ((string s) => Debug.Log(s),        // Callback; Here, it just logs it to the Debug Log
                    () => Debug.LogError("Error!"), // Error callback
                    10000f                          // Timeout (milliseconds)
                )
            );
        }
    }
}