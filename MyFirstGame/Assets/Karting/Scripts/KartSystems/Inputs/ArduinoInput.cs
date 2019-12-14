using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KartGame.KartSystems
{

    /// <summary>
    /// An implementation of the IInput interface for all the input information a kart needs
    /// using our custom haptic feedback device.
    /// 
    /// Our device allows for: acceleration, braking, and steering
    /// </summary>
    public class ArduinoInput : MonoBehaviour, IInput
    {
        public ArduinoThread thread;

        [RequireInterface(typeof(IKartInfo))]
        public Object kartInfo;

        IKartInfo m_KartInfo;

        // Speed of the race car to be sent to the interface
        public float velz;

        public float Acceleration
        {
            get { return m_Acceleration; }
        }
        public float Steering
        {
            get { return m_Steering; }
        }
        public bool BoostPressed
        {
            get { return m_BoostPressed; }
        }
        public bool FirePressed
        {
            get { return m_FirePressed; }
        }
        public bool HopPressed
        {
            get { return m_HopPressed; }
        }
        public bool HopHeld
        {
            get { return m_HopHeld; }
        }

        float m_Acceleration;
        float m_Steering;
        bool m_HopPressed;
        bool m_HopHeld;
        bool m_BoostPressed;
        bool m_FirePressed;

        bool m_FixedUpdateHappened;

        void Awake()
        {
            thread.StartThread();
            m_KartInfo = kartInfo as IKartInfo;
        }
        void Update()
        {
            // If using the connector instead of threading, use this code to read from Arduino:
            /* StartCoroutine(
                         connector.AsynchronousReadFromArduino
                        (
                          (string s) => Debug.Log(s),     // Callack
                           () => Debug.LogError("Error!"), // Log Error
                           10f                          // Timeout (milliseconds)
                         )*/


            // Provide important input information to the game from the Arduino.

            // Prompt Arduino for signal.
            velz = m_KartInfo.Velocity.z;
            string s_acc = thread.ReadAccFromArduino();
            
            // Acceleration vs Braking Control
            if (s_acc != null)          // if there is something to be read
            {
                if (s_acc.Contains("+")) // accelerate
                {
                    m_Acceleration = 1f;
                    // Debug.Log("+");
                }
                else if ((s_acc.Contains("-")) && (velz > 0)) // brake; with non-reversing condition
                {
                    m_Acceleration = -1f;
                    // Debug.Log("-");
                }
                else //s_acc null
                {
                    m_Acceleration = 0f;
                }
            }
            else    // if s_acc is null / queue length is 0
            {
                //
                // do nothing
            }
            // Keyboard Implementation
            //if (Input.GetKey(KeyCode.UpArrow))
            //    m_Acceleration = 1f;
            //else if (Input.GetKey(KeyCode.DownArrow))
            //    m_Acceleration = -1f;
            //else
            //    m_Acceleration = 0f;

            // 2. STEERING

            // Retrieve Steering Information


            // Steering: L --> Left, R --> right
            string s_str = thread.ReadSteerFromArduino();
            if (s_str != null)                  // if tere is something to be read
            {
                if (s_str.Contains("L")) // accelerate
                {
                    m_Steering = -1f;
                   //  Debug.Log("L");
                }
                else if (s_str.Contains("R")) // brake; with non-reversing condition
                {
                    m_Steering = 1f;
                    // Debug.Log("R");
                }
                else
                {
                    m_Steering = 0f;
                }
            } else
            {
                // do nothing
            }
            // Keyboard Implementation:
            //if (Input.GetKey(KeyCode.F) && !Input.GetKey(KeyCode.H))
            // m_Steering = -1f;
            //else if (!Input.GetKey(KeyCode.F) && Input.GetKey(KeyCode.H))
            // m_Steering = 1f;
            // else
            // m_Steering = 0f;

            //  Game Pad Implementation:
            //m_Steering = Input.GetAxis("Horizontal"); // range needs to be mapped from -1 to 1?

            // Keep the Keyboad input methods for the rest of the input modes
            // These inputs are not implemented on our device.

            m_HopHeld = Input.GetKey(KeyCode.Space); 

            if (m_FixedUpdateHappened)
            {
                m_FixedUpdateHappened = false;

                m_HopPressed = false;
                m_BoostPressed = false;
                m_FirePressed = false;
            }

            m_HopPressed |= Input.GetKeyDown(KeyCode.Space);
            m_BoostPressed |= Input.GetKeyDown(KeyCode.RightShift);
            m_FirePressed |= Input.GetKeyDown(KeyCode.RightControl);
        }
    
        void FixedUpdate()
        {
            m_FixedUpdateHappened = true;
        }

        void OnDestroy()
        {
            thread.StopThread();
        }

    }    
}