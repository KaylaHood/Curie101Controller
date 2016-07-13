using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

public class CurieSerialReader
{
    // -----------------------------
    // Private Variable Declarations
    // -----------------------------

    // debug boolean
    private bool debug = false;

    //private string logFileName = "logfile.txt";
    //private System.IO.FileStream logFile;
    private SerialPort serial;
    // size, in bytes, of message sent to serial port by Curie
    private int serialMessageSize = 22;

    // enums for easy access of special integer values
    private enum Opcodes { normal, zeromotion, calibrate };
    private enum Orientations { FaceUp, FaceDown, DigitalUp, AnalogUp, ACDown, ACUp }

    // range constants (received from Curie after calibration)
    private float CurieAccelerometerRange;
    private float CurieGyroscopeRange;

    // time keeping variable (for calculating dt)
    private ulong previousTime;

    // storage variables for angle correction
    private float lastAccurateZ;
    private float lastAccurateY;
    private float lastAccurateX;
    private int lastOrientation;

    // private location data (not needed by any other operation outside of this class)
    private Vector3 estRotOffset;
    private Vector3 originalGravity;
    private Vector3 gravity;

    // ----------------------------
    // Public Variable Declarations
    // ----------------------------

    // serial message object for serial port management
    public Message serialMsg;

    // public location data (used for moving object within scene)
    public Vector3 acc;
    public Quaternion kalmanCorrectedRot;
    public Quaternion madgwickRot;

    // zero motion boolean (implementing classes can decide to use this or not)
    public bool isZeroMotion;

    // Single-Dimensional Kalman Filters
    public Kalman kalmanX;
    public Kalman kalmanY;
    public Kalman kalmanZ;
    
    // Three-Dimensional Madgwick Filter
    public AHRS.MadgwickAHRS madgwick;

    // UI text components for display of demo data
    public UnityEngine.UI.Text trackingInfo = null;
    // string to put in "trackingInfo"
    string info;

    // --------------------------
    // Interior Class Definitions
    // --------------------------

    // Message Class for easy management of data from Curie

    public class Message
    {
        private SerialPort serial;
        private int messageLength;
        private byte[] message;
        private string calibrationMsg = "c";
        private string beginMsg = "y";
        private string disconnectionMsg = "d";
        public string msgString;
        public short opcode;
        public ulong timestamp;
        public Vector3 rawAcc;
        public Vector3 rawGyro;

        public Message(SerialPort s, int msgLen)
        {
            // Set up variables
            serial = s;
            messageLength = msgLen;
            message = new byte[msgLen];
            rawAcc = Vector3.zero;
            rawGyro = Vector3.zero;
        }

        public void RequestCalibration()
        {
            serial.Write(calibrationMsg);
        }
        public void Begin()
        {
            serial.Write(beginMsg);
        }
        public void Disconnect()
        {
            serial.Write(disconnectionMsg);
        }

        public void sendMessageToSerial()
        {
            // send message back to serial port
            serial.Write(message, 0, messageLength);
        }

        public void ReadAndUpdate()
        {
            // Read new data from serial port and divide into values
            readBytesFromSerial();
            updateOpcode();
            updateTimestamp();
            updateAcc();
            updateGyro();
        }

        public void readBytesFromSerial()
        {
            for (int i = 0; i < messageLength; i++)
            {
                message[i] = (byte)serial.ReadByte();
            }
        }

        public short updateOpcode()
        {
            opcode = BitConverter.ToInt16(message, 0);
            return opcode;
        }

        public ulong updateTimestamp()
        {
            timestamp = BitConverter.ToUInt64(message, 2);
            return timestamp;
        }

        public Vector3 updateAcc()
        {
            rawAcc.x = (float)BitConverter.ToInt16(message, 10);
            rawAcc.y = (float)BitConverter.ToInt16(message, 12);
            rawAcc.z = (float)BitConverter.ToInt16(message, 14);
            return rawAcc;
        }

        public Vector3 updateGyro()
        {
            rawGyro.x = (float)BitConverter.ToInt16(message, 16);
            rawGyro.y = (float)BitConverter.ToInt16(message, 18);
            rawGyro.z = (float)BitConverter.ToInt16(message, 20);
            return rawGyro;
        }

        public string getString()
        {
            msgString = BitConverter.ToString(message);
            return msgString;
        }

        public void DiscardInBuffer()
        {
            serial.DiscardInBuffer();
        }

    }

    public class BLEMessage
    {
        //~~~~**************~~~~
        //~~~~****_TODO_****~~~~
        //~~~~**************~~~~
        public byte[] message;
        short opcode;
        ulong timestamp;
        Vector3 rawAcc;
        Vector3 rawGyro;

        public BLEMessage(byte[] m,short op)
        {
            message = m;
            opcode = op;
            // do work to set other variables
        }
    }

    // --------------------------------------------
    // CurieSerialReader Class Function Definitions
    // --------------------------------------------
        
    public CurieSerialReader(string port)
    {
        // Setup serial port
        serial = new SerialPort(
        port, 9600, Parity.None, 8, StopBits.One
        );    
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 1000;

        // allow time for setup. Unity has issues without this.
        Thread.Sleep(100);

        // Setup serial port managing object
        serialMsg = new Message(serial, serialMessageSize);

        // send opening message to serial port
        serialMsg.Begin();

        // Initialize your global acceleration and rotation variables
        acc = Vector3.zero;
        kalmanCorrectedRot = Quaternion.identity;
        madgwickRot = Quaternion.identity;
  
        // initialize Kalman filters
        kalmanX = new Kalman();
        kalmanY = new Kalman();
        kalmanZ = new Kalman();

        // initialize Madgwick filter (samplePeriod is in seconds, our sample periods vary so we must set the
        // sample period each time we update the madgwick filter)
        madgwick = new AHRS.MadgwickAHRS(1.0f);
    }
    public CurieSerialReader(SerialPort port)
    {
        // Setup your port object.
        serial = port;
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 1000;

        // allow time for setup. Unity has issues without this.
        Thread.Sleep(100);

        // Setup serial port managing object
        serialMsg = new Message(serial, serialMessageSize);

        // send opening message to serial port
        serialMsg.Begin();

        // Setup your acceleration and rotation vectors
        acc = Vector3.zero;
        kalmanCorrectedRot = Quaternion.identity;
        madgwickRot = Quaternion.identity;

        // initialize Kalman filters
        kalmanX = new Kalman();
        kalmanY = new Kalman();
        kalmanZ = new Kalman();

        // initialize Madgwick filter (samplePeriod is in seconds, our sample periods vary so we must set the
        // sample period each time we update the madgwick filter)
        madgwick = new AHRS.MadgwickAHRS(1.0f);
    }

    public void UpdateValues()
    {
        // use these later for converted acc and gyro values
        Vector3 convAcc = Vector3.zero;
        Vector3 convGyro = Vector3.zero;
        Vector3 kalmanRot = Vector3.zero;

        // update serial port object
        serialMsg.ReadAndUpdate();

        // set the UI "info" string to the received statement 
        // (using assignment operator to clear previous values)
        info = "Received this statement: " + serialMsg.msgString + "\n";

        info += ("Values interpreted: " + serialMsg.opcode + " | " + serialMsg.timestamp + " | " + serialMsg.rawAcc 
            + " | " + serialMsg.rawGyro + "\n");

        // set boolean for zero motion detection
        isZeroMotion = (serialMsg.opcode == (int)Opcodes.zeromotion);

        // convert raw values into G's
        convAcc.Set(
            convertRawAcceleration(serialMsg.rawAcc.x),
            convertRawAcceleration(serialMsg.rawAcc.y),
            convertRawAcceleration(serialMsg.rawAcc.z)
            );
        // convert raw values into radians/time
        convGyro.Set(
            convertRawGyroRad(serialMsg.rawGyro.x),
            convertRawGyroRad(serialMsg.rawGyro.y),
            convertRawGyroRad(serialMsg.rawGyro.z)
            );

        if (serialMsg.opcode == (int)Opcodes.calibrate)
        {
            // set gravity vector (after conversion)
            // this is done because whatever forces are felt by the curie
            // at calibration (when it is flat and stationary) will
            // coorespond to the force of gravity in its frame of reference
            originalGravity = convAcc;
        }

        // estimate rotation angles based on accelerometer values
        // roll = x, pitch = y, yaw = z
        Vector3 estRot = Vector3.zero;
        estRot.x = Mathf.Atan2(convAcc.y, convAcc.z) * Mathf.Rad2Deg;
        estRot.y = Mathf.Atan2(convAcc.x, convAcc.z) * Mathf.Rad2Deg;
        estRot.z = Mathf.Atan2(convAcc.x, convAcc.y) * Mathf.Rad2Deg;

        // set Kalman filters for rotations
        kalmanX.setAngle(estRot.x);
        kalmanY.setAngle(estRot.y);
        kalmanZ.setAngle(estRot.z);

        // calculate delta time from last update
        float dt;
        if (previousTime > serialMsg.timestamp)
        {
            // the timer has overflowed back to zero 
            // (unsigned 32-bit value overflows at 4294967295, which is about 70 minutes
            // since the timer is in microseconds)
            float tmp = previousTime;
            tmp -= uint.MaxValue;
            dt = serialMsg.timestamp - tmp;
        }
        else
        {
            dt = serialMsg.timestamp - previousTime;
        }
        // set previous time now that the dt has been calculated
        previousTime = serialMsg.timestamp;
        // change dt to seconds from microseconds
        // multiply by the DLPF setting (currently 4x low pass filter)
        dt = (dt / 10000000.0f) * 4.0f;

        // do Kalman filtering for rotation
        kalmanRot.x = kalmanX.getAngle(estRot.x, convGyro.x, dt);
        kalmanRot.y = kalmanY.getAngle(estRot.y, convGyro.y, dt);
        kalmanRot.z = kalmanZ.getAngle(estRot.z, convGyro.z, dt);

        // correct angle according to board's orientation
        kalmanCorrectedRot = Quaternion.Euler(angleCorrection(getOrientation(serialMsg.rawAcc), kalmanRot));

        // do Madgwick filtering, set sample period each time because our sample rate 
        // is somewhat inconsistent
        madgwick.SamplePeriod = dt;
        madgwick.Update(convGyro.x, convGyro.y, convGyro.z, convAcc.x, convAcc.y, convAcc.z);

        // get Quaternion from Madgwick filter
        madgwickRot.Set(madgwick.Quaternion[0], madgwick.Quaternion[1], madgwick.Quaternion[2], madgwick.Quaternion[3]);

        // rotate gravity vector using madgwick filter rotation
        gravity = madgwickRot * originalGravity;

        // set acc to be the translational acceleration
        acc = convAcc - gravity;

        // write data to UI text elements
        info += ("timestep: " + dt + "\nestimated rotation :" + estRot + "\nkalman rotation: " + kalmanRot + "\ncorrected kalman rotation: " +
            kalmanCorrectedRot.eulerAngles + "\nmadgwick rotation: " + madgwickRot.eulerAngles + "\nconverted acceleration: " + convAcc +
            "\nmadgwick gravity: " + gravity + "\nacc after gravity is subtracted: " + acc);
        trackingInfo.text = info;
   }

    // **Calibration should only need to be run once per Curie unit
    public bool Calibrate()
    {
        int loops = 0;
        int tries = 0;

        // Wait until the "calibration complete" opcode is received
        while ((serialMsg.opcode != (int)Opcodes.calibrate) && (tries < 20))
        {
            if (debug)
            {
                Debug.Log("try: " + tries);
            }
            // ~~~ Request Calibration ~~~
            serialMsg.DiscardInBuffer();
            serialMsg.RequestCalibration();
            while ((serialMsg.opcode != (int)Opcodes.calibrate) && (loops < 100))
            {
                if (debug)
                {
                    Debug.Log("loop: " + loops);
                    Debug.Log("Not calibration: " + serialMsg.getString());
                }
                serialMsg.ReadAndUpdate();
                loops += 1;
            }
            tries += 1;
            loops = 0;
        }
        if (serialMsg.opcode != (int)Opcodes.calibrate)
        {
            return false;
        }
        else
        {
            if (debug)
            {
                Debug.Log("Calibration: " + serialMsg.getString());
            }
            // get range values from Curie (printed only just after calibration)
            CurieAccelerometerRange = serialMsg.rawGyro.x;
            CurieGyroscopeRange = serialMsg.rawGyro.y;
            return true;
        }
    }

    public void Disconnect()
    {
        // send exit message to serial port
        serialMsg.Disconnect();
    }

    float convertRawAcceleration(float aRaw) {
        // since we are using Gs
        // -(max range) maps to a raw value of -32768
        // +(max range) maps to a raw value of 32767

        float a = (aRaw * CurieAccelerometerRange) / 32768.0f;

        return a;
    }

    float convertRawGyroDeg(float gRaw) {
        // since we are using degrees/seconds
        // -(max range) maps to a raw value of -32768
        // +(max range) maps to a raw value of 32767

        float g = (gRaw * CurieGyroscopeRange) / 32768.0f;

        return g;
    }

    float convertRawGyroRad(float gRaw) {
        float g = convertRawGyroDeg(gRaw);
        // convert to radians/second
        g *= Mathf.PI;
        g /= 180.0f;
        return g;
    }

    int getOrientation(Vector3 rawAcc)
    {
        // 0 = board face up
        // 1 = board face down
        // 2 = digital pins up
        // 3 = analog pins up
        // 4 = AC facing down
        // 5 = AC facing up
        // these values are represented in the enum "orientation"
        int orientation;
        if(Mathf.Abs(rawAcc.z) > Mathf.Abs(rawAcc.y) && Mathf.Abs(rawAcc.z) > Mathf.Abs(rawAcc.x))
        {
            if (rawAcc.z > 0)
            {
                orientation = (int)Orientations.FaceUp;
            }
            else
            {
                orientation = (int)Orientations.FaceDown;
            }
        } 
        else if(Mathf.Abs(rawAcc.y) > Mathf.Abs(rawAcc.x) && Mathf.Abs(rawAcc.y) > Mathf.Abs(rawAcc.z))
        {
            if (rawAcc.y > 0)
            {
                orientation = (int)Orientations.DigitalUp;
            }
            else
            {
                orientation = (int)Orientations.AnalogUp;
            }
        } 
        else
        {
            if (rawAcc.x > 0)
            {
                orientation = (int)Orientations.ACDown;
            }
            else
            {
                orientation = (int)Orientations.ACUp;
            }

        } 
        return orientation;
    }

    float diffEulerAngles(float a1, float a2)
    {
        // returns difference between angles which have a range of (-180,180)
        // this means that -180 = 180, because the angle values roll over when
        // they exceed the range limits.
        // for example, the difference between -170 and 170 is -20
        // the returned float, "diff", will be such that :
        // a1 - diff = a2
        float diff = a1 - a2;
        if ((a1 > 0 && a2 < 0) && (Mathf.Abs((180.0f - a1) + (a2 + 180.0f)) < Mathf.Abs(diff)))
        {
            // store negative value because if a1 needs to roll over 180 to become
            // a2, then it must be added to, so diff must be negative (see equation above)
            diff = -((180.0f - a1) + (a2 + 180.0f));
        }
        else if ((a1 < 0 && a2 > 0) && (Mathf.Abs((180.0f - a2) + (a1 + 180.0f)) < Mathf.Abs(diff)))
        {
            diff = (180.0f - a2) + (a1 + 180.0f);
        }
        return diff;
        
    }

    float diffFromFlat(int orientation,Vector3 rot)
    {
        // returns the difference between the rotation passed in and 
        // the "flat" rotation for the orientation.
        float modifier;
        if (orientation == (int)Orientations.FaceUp)
        {
            // flat is (0,0,?)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, 0.0f)) + Mathf.Abs(diffEulerAngles(rot.y, 0.0f));
            info += ("Difference of " + rot + " from (0,0,?): " + modifier + "\n");
        }
        else if (orientation == (int)Orientations.FaceDown)
        {
            // flat is (180,180,?)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, 180.0f)) + Mathf.Abs(diffEulerAngles(rot.y, 180.0f));
            info += ("Difference of " + rot + " from (180,180,?): " + modifier + "\n");
        }
        else if (orientation == (int)Orientations.DigitalUp)
        {
            // flat is (90,?,0)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, 90.0f)) + Mathf.Abs(diffEulerAngles(rot.z, 0.0f));
            info += ("Difference of " + rot + " from (90,?,0): " + modifier + "\n");
        }
        else if (orientation == (int)Orientations.AnalogUp)
        {
            // flat is (-90,?,180)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, -90.0f)) + Mathf.Abs(diffEulerAngles(rot.z, 180.0f));
            info += ("Difference of " + rot + " from (-90,?,180): " + modifier + "\n");
        }
        else if(orientation == (int)Orientations.ACDown)
        {
            // flat is (?,90,90)
            modifier = Mathf.Abs(diffEulerAngles(rot.y, 90.0f)) + Mathf.Abs(diffEulerAngles(rot.z, 90.0f));
            info += ("Difference of " + rot + " from (?,90,90): " + modifier + "\n");
        }
        else
        {
            // flat is (?,-90,-90)
            modifier = Mathf.Abs(diffEulerAngles(rot.y, -90.0f)) + Mathf.Abs(diffEulerAngles(rot.z, -90.0f));
            info += ("Difference of " + rot + " from (?,-90,-90): " + modifier + "\n");
        }
        return modifier;
    }

    Vector3 angleCorrection(int orientation, Vector3 rot)
    {
        // This function uses the orientation to determine which axis is
        // inaccurate due to the position of the board.
        // For example, when the board is flat for calibration, the z-axis
        // rotation angle is not able to be estimated using the accelerometer
        // data, and thus we cannot rely on that axis' measurement.
        //
        // "orientation" should be the int returned by calling "getOrientation"
        //
        // "rot" is the rotation vector being corrected for inaccuracies
        //
        // "lastAccurate_" holds the value which the inaccurate axis will approach as it becomes
        // less accurate (either the x, y, or z value depending on which is the inaccurate axis).
        //
        Vector3 newRot = rot;
        // calculate modifier, which is the difference between the "accurate" angles and
        // either 0, 90, 180, -90, or -180, depending on which value the angle is closest to.
        // **this logic is done in function "diffFromFlat"
        float modifier = diffFromFlat(orientation, rot);            
        if(orientation == (int)Orientations.FaceUp || orientation == (int)Orientations.FaceDown)
        {
            // board face up or face down (z-axis is inaccurate)
            info += ("z-axis is inaccurate\n");
            // set last accurate angle if orientation has changed
            if(lastOrientation != orientation)
            {
                // use "kalmanCorrectedRot" because it *should* contain previous update's rotation value
                lastAccurateZ = kalmanCorrectedRot.eulerAngles.z;
            }
            info += ("last accurate z: " + lastAccurateZ + "\n");
            // if modifier is less than 1, the correction will overshoot the "last accurate Z"
            // so only do the gradual correction when the modifier is greater than one
            /*if (modifier >= 1)
            {
                // calculate difference between unmodified rotation and the "last accurate Z"
                float diff = diffEulerAngles(rot.z,lastAccurateZ);
                // reduce magnitude of rotation value
                // (use log base 1.3 because it provides best logarithmic scale for data -- base can be tweaked)
                float newZ = rot.z - (diff / Mathf.Log(modifier,1.3f));
                newRot.z = newZ;
            }
            else
            {*/
                //newRot.z = lastAccurateZ;
                newRot.z = 0.0f; // Kayla Debug
            //}
        }
        else if(orientation == (int)Orientations.DigitalUp || orientation == (int)Orientations.AnalogUp)
        {
            // digital pins up or analog pins up (y-axis is inaccurate)
            info += ("y-axis is inaccurate\n");
            if(lastOrientation != orientation)
            {
                lastAccurateY = kalmanCorrectedRot.y;
            }
            info += ("last accurate y: " + lastAccurateY + "\n");
            /*if (modifier >= 1)
            {
                float diff = diffEulerAngles(rot.y, lastAccurateY);
                float newY = rot.y - (diff / Mathf.Log(modifier,1.3f));
                newRot.y = newY;
            }
            else
            {*/
                //newRot.y = lastAccurateY;
                newRot.y = 0.0f; // Kayla Debug
            //}
        }
        else if(orientation == (int)Orientations.ACDown || orientation == (int)Orientations.ACUp) 
        {
            // AUX facing down or facing up (x-axis is inaccurate)
            info += ("x-axis is inaccurate\n");
            if(lastOrientation != orientation)
            {
                lastAccurateX = kalmanCorrectedRot.x;
            }
            info += ("last accurate x: " + lastAccurateX + "\n");
            /*if (modifier >= 1)
            {
                float diff = diffEulerAngles(rot.x, lastAccurateX);
                float newX = rot.x - (diff / Mathf.Log(modifier,1.3f));
                newRot.x = newX;
            }
            else
            {*/
                //newRot.x = lastAccurateX;
                newRot.x = 0.0f; // Kayla Debug
            //}
        }
        lastOrientation = orientation;
        return newRot;
    }
}
