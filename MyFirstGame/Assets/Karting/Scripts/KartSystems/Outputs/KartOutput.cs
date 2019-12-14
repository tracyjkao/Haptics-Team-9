using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace KartGame.KartSystems
{
    public class KartOutput : MonoBehaviour
    {
        [RequireInterface(typeof(IKartInfo))]
        public Object kartInfo;

        // public ArduinoConnector connector;
        public ArduinoThread thread;

        IKartInfo m_KartInfo;

        // Speed of the race car to be sent to the interface
        public float speed;
        // X position of the car.
        public float xposition;
        private float xoffset = 2;       // Distance away before vibration ("into the shoulder zone")
        private float startingxpos = 16;

        private int frametracker = 0;       // Track number of frames created
        public int framemod = 1;            // Number of frames per serial communication to Arduino

        // Awake is called at initialization
        void Awake()
        {
            // Connect Kart Information
            m_KartInfo = kartInfo as IKartInfo;
        }

        // Start is called before the first frame update
        private void Start()
        {
            // Any Prep Work
        }

        // Update is called once per frame
        void Update()
        {
            // Send information every n frames
            frametracker++;                 // Increment every frame
            frametracker %= frametracker;   // Make tracker between 0 and n    

            // Check if it's the nth frame
            if (frametracker == 0)          // If n frames have been reached
            {
                // Send following information to Arduino:
                
                // 1. Speed/Pos Combination 
                //speed = m_KartInfo.LocalSpeed;
                //thread.SendSpToArduino(speed.ToString());

                // 2. Pos Combination 
                xposition = m_KartInfo.Position.x;

                // thread.SendPosToArduino(xposition.ToString());
                // Debug.Log("xposition is:" + xposition);
                // Debug.Log("xposition right border is:" + (startingxpos + xoffset));
                //Debug.Log("xposition > right border?" + (xposition > (startingxpos + xoffset)));

                // 3. Vibration Information (along a straight track)
                // If vehicle enters the left "shoulder" of the road
                if (m_KartInfo.Velocity.z < 0)
                {
                    thread.SendVibrToArduino("B");
                } else if (xposition < (startingxpos - xoffset))
                {
                    // Debug.Log("Decide to Vibrate L Motor");
                    thread.SendVibrToArduino("L");    // Signal to actuate L vibration motor.     
                }
                // if the vehicle enters the right "shoulder" of the road
                else if (xposition > (startingxpos + xoffset))
                {
                    // Debug.Log("Decide to Vibrate R Motor");
                    thread.SendVibrToArduino("R");    // Signal to actuate R vibration motor.
                }
                // if the vehicle is in the range of the road
                else
                {
                    thread.SendVibrToArduino("O");     // Signal to turn off existing vibration.
                }
                
                // All information sent to Arduino.
            }
            else
            {
                // do nothing
            }
        }
    }
}