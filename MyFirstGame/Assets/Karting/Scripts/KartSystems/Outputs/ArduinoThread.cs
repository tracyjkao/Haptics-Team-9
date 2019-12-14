using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;
using System.IO.Ports;

namespace KartGame.KartSystems
{

    public class ArduinoThread : MonoBehaviour
    {
        private Thread thread;
        public ArduinoConnector connector;

        // Input Queues (From Arduino)
        private Queue accInputQueue;   // Kart movement commands (acceleration)
        private Queue steerInputQueue; // The steering wheel position (steer L/R)
       
        // Output Queues (To Arduino)
        private Queue speedOutputQueue; // To modulate intensity of fan
        private Queue posOutputQueue;   // To enable the virtual wall.
        private Queue vibrOutputQueue;  // To modulate vibration

        // Queue Lengths
        public int accqlen = 0;
        public int strinqlen = 0;
        public int spoutqlen = 0;
        public int posoutqlen = 0;
        public int vibroutqlen = 0;

        // Latest commands (From Arduino)
        private string lastAccInput;
        private string lastSteerInput;
       
        // Latest commands (To Arduino)
        private string lastSpeedOutput;
        private string lastPosOutput;
        private string lastVibrOutput;

        private string outputMessage;

        // private int timeout = 50;
        private bool looping = true;
        public void StartThread()
        {
            Debug.Log("Starting Thread");

            // Start all the queues in the thread
            accInputQueue = Queue.Synchronized(new Queue());
            steerInputQueue = Queue.Synchronized(new Queue());
            speedOutputQueue = Queue.Synchronized(new Queue());
            posOutputQueue = Queue.Synchronized(new Queue());
            vibrOutputQueue = Queue.Synchronized(new Queue());

            // outputQueue = Queue.Synchronized(new Queue());
            // inputQueue = Queue.Synchronized(new Queue()); 

            // Check Which Ports are Available
            foreach (string str in SerialPort.GetPortNames())
            {
                // Debug.Log(string.Format("Existing COM port: {0}", str));
            }

            // Opens the connector
            connector.Open();

            // Creates and starts the thread
            thread = new Thread(ThreadLoop);
            thread.Start();
        }

        // Methods to Send Information to Arduino
        public void SendSpToArduino(string speed)  // Send Combined Speed and Position Value to Arduino
        {
            lastSpeedOutput = speed;
            //speedOutputQueue.Enqueue("S");// Initiation signal
            //speedOutputQueue.Enqueue(speed);
            //speedOutputQueue.Enqueue("T"); // Termination signal
            //spoutqlen += 3;
        }
        public void SendPosToArduino(string position)
        {
            lastPosOutput = position;
            //posOutputQueue.Enqueue("P"); // Initiation signal
            //posOutputQueue.Enqueue(position);
            //posOutputQueue.Enqueue("Q"); // Termination signal
            //posoutqlen += 3;
        }
        public void SendVibrToArduino(string command)   // Send Vibration Command to Arduino
        {
            lastVibrOutput = command;
            //vibrOutputQueue.Enqueue(command);
            //vibroutqlen++;
        }

        // Methods to Read Information from Arduino
        public string ReadAccFromArduino()     // Dequeues the Steering Input Queue
        {
            /* if (accInputQueue.Count == 0)       
            {
                Debug.Log("No ThreadReadFromArduino");
                return null;
            }
            else
            {
                accqlen--;
                return (string)accInputQueue.Dequeue();
            } */
            return lastAccInput;
        }

        public string ReadSteerFromArduino()    // Dequeues the Steering Input Queue
        {
            /* if (steerInputQueue.Count == 0)
            {
                return null;
            }
            else
            {
                strinqlen--;
                return (string)steerInputQueue.Dequeue();
            } */
            return lastSteerInput;
        }

//        public void ThreadSendToArduino(string command)
//        {
//            outputQueue.Enqueue(command);
//            outquelen++;
//        }

//        public string ThreadReadFromArduino()
//        {
//            if (inputQueue.Count == 0)
//            {
//                // Debug.Log("No ThreadReadFromArduino");
//                return null;
//            }
//            inquelen--;
//            return (string)inputQueue.Dequeue();
//        }

        public void ThreadLoop()
        {

            // Looping
            while (IsLooping())
            {
                // Process Each Queue

                // Send to Arduino:
                // Next Speed Item (To Actuate Fan
                //if (speedOutputQueue.Count != 0)
                //{
                // outputMessage = "S" + lastSpeedOutput;


                // outputMessage = lastSpeedOutput;
                // connector.WriteToArduino(outputMessage);
                // Debug.Log("Write S to Arduino");
                // connector.WriteToArduino(outputMessage);
                // Debug.Log("Write " + outputMessage + " to Arduino");
                // connector.WriteToArduino("T");
                // Debug.Log("Write T to Arduino.");


                //string command = (speedOutputQueue.Dequeue()).ToString(); // Output "S"
                //spoutqlen--;
                //connector.WriteToArduino(command);
                //string message = (speedOutputQueue.Dequeue()).ToString(); // Output speed information
                //spoutqlen--;
                //connector.WriteToArduino(message);
                //string terminal = (speedOutputQueue.Dequeue()).ToString(); // Output "T"
                //spoutqlen--;
                //connector.WriteToArduino(terminal);
                //}

                // Next Position Item
                //if (posOutputQueue.Count != 0)
                //{
                // outputMessage = "P" + lastPosOutput;
                // outputMessage = lastPosOutput;
                // connector.WriteToArduino("P");
                // connector.WriteToArduino(lastPosOutput);
                // connector.WriteToArduino("Q");
                //string command = (posOutputQueue.Dequeue()).ToString(); // Output "P"
                //posoutqlen--;
                //connector.WriteToArduino(command);
                //string message = (posOutputQueue.Dequeue()).ToString(); // Output speed information
                //posoutqlen--;
                //connector.WriteToArduino(message);
                //string terminal = (posOutputQueue.Dequeue()).ToString(); // Output "Q"
                //posoutqlen--;
                //connector.WriteToArduino(terminal);
                //}

                // Next Vibrate Item
                //if (vibrOutputQueue.Count != 0)
                //{
                // string command = (vibrOutputQueue.Dequeue()).ToString();
                // vibroutqlen--;
                connector.WriteToArduino(lastVibrOutput);
                // Debug.Log("WriteVibrationToArduino: "+lastVibrOutput);
                
                //}

                // if (outputQueue.Count != 0)
                // {
                // string command = outputQueue.Dequeue().ToString();
                // outquelen--;
                // connector.WriteToArduino(command);
                // }

                // Read from Arduino
                string result = connector.ReadFromArduino(80); // timeout = 0?
                // result = connector.ReadFromArduino(0);
                // result = connector.ReadFromArduino(0);
                // Debug.Log("Serial reading is: " + result);
                if (result != null)
                {
                    // Categorize the input into the appropriate Queue
                    // Debug.Log("Result is:" + result);
                    // If the input is an Acceleration Command 
                    if (result.Contains("+") || result.Contains("-"))
                    {
                        // accInputQueue.Enqueue(result);
                        // accqlen++;
                        lastAccInput = result;
                    }
                    // If the input is a Steering Value
                    if ((result.Contains("R")) || (result.Contains("L")))
                    {
                        // Debug.Log("Steer Command:" + result);
                        lastSteerInput = result;
                        // steerInputQueue.Enqueue(result);
                        // strinqlen++;    

                    }
                    // If the input is neither Acceleration nor Steering
                    else
                    {
                        // Debug.Log(result);
                        // invalid input
                        // move on to next input
                    }



                    // inputQueue.Enqueue(result);
                    // inquelen++;
                }

            }

        }

        public bool IsLooping()
        {
            lock (this)
            {
                return looping;
            }
        }
        public void StopThread()
        {
            connector.WriteToArduino("E");
            Debug.Log("Stopping Thread.");
            connector.Close();
            
            

            lock (this)
            {
                looping = false;
            }
           
        }

    }
}
