using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

//Create a new SerialPort property in your class.

public class CurieSerialReader
{
    //private string logFileName = "logfile.txt";
    //private System.IO.FileStream logFile;
    private SerialPort serial;
    private enum Opcodes { calibrate, normal, zeromotion };
    private enum Orientations { FaceUp, FaceDown, DigitalUp, AnalogUp, ACDown, ACUp }
    private float CurieAccelerometerRange;
    private float CurieGyroscopeRange;
    private long previousTime;
    private Vector3 estRotOffset;
    private Vector3 originalGravity;
    public Vector3 acc;
    public Quaternion kalmanCorrectedRot;
    public Quaternion madgwickRot;
    public Vector3 gravity;
    public bool isZeroMotion;
    // Single-Dimensional Kalman Filters
    Kalman kalmanX;
    Kalman kalmanY;
    Kalman kalmanZ;
    // Three-Dimensional Madgwick Filter
    AHRS.MadgwickAHRS madgwick;

    // UI text components for display of demo data
    public UnityEngine.UI.Text trackingInfo = null;
    // string to put in "trackingInfo"
    string info;

    public CurieSerialReader(string port)
    {
        // Setup your port object.
        serial = new SerialPort(
        port, 9600, Parity.None, 8, StopBits.One
        );
        // Setup your acceleration and rotation vectors
        acc = Vector3.zero;
        kalmanCorrectedRot = Quaternion.identity;
        madgwickRot = Quaternion.identity;
  
        // Set additional parameters.    
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 1000;

        // allow time for setup. Unity has issues without this.
        Thread.Sleep(100);

        // initialize Kalman filters
        kalmanX = new Kalman();
        kalmanY = new Kalman();
        kalmanZ = new Kalman();

        // initialize Madgwick filter (samplePeriod is in seconds, our sample periods vary so we must set the
        // sample period each time we update the madgwick filter)
        madgwick = new AHRS.MadgwickAHRS(1.0f);

        // set up log file
        //logFile = System.IO.File.Open(logFileName, System.IO.FileMode.OpenOrCreate, System.IO.FileAccess.Write);
    }
    public CurieSerialReader(SerialPort port)
    {
        // Setup your port object.
        serial = port;
        // Setup your acceleration and rotation vectors
        acc = Vector3.zero;
        kalmanCorrectedRot = Quaternion.identity;
        madgwickRot = Quaternion.identity;

        // Set additional parameters.    
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 50000;

        // allow time for setup. Unity has issues without this.
        Thread.Sleep(100);

        // initialize Kalman filters
        kalmanX = new Kalman();
        kalmanY = new Kalman();
        kalmanZ = new Kalman();

        // initialize Madgwick filter (samplePeriod is in seconds, our sample periods vary so we must set the
        // sample period each time we update the madgwick filter)
        madgwick = new AHRS.MadgwickAHRS(1.0f);

        // set up log file
        //logFile = System.IO.File.Open(logFileName, System.IO.FileMode.OpenOrCreate, System.IO.FileAccess.Write);
    }

    public void UpdateValues(string statement = "")
    {
        // You may pass-in a pre-read statement, for special uses.
        // For example, this code uses this argument for passing in the
        // calibration statement after requesting calibration.
        //
        // empty info string
        info = "";
        // use these later for converted acc and gyro values
        Vector3 convAcc = Vector3.zero;
        Vector3 convGyro = Vector3.zero;
        Vector3 kalmanRot = Vector3.zero;
        if (statement == "")
        {
            statement = serial.ReadTo(";");
        }
        string[] values = statement.Split(',');
        // parse separated raw values into separate variables
        Vector3 rawAcc = new Vector3(
            int.Parse(values[2]), 
            int.Parse(values[3]), 
            int.Parse(values[4])
            );
        Vector3 rawGyro = new Vector3(
            int.Parse(values[5]),
            int.Parse(values[6]),
            int.Parse(values[7])
            );
        int opcode = int.Parse(values[0]);
        long timestamp = long.Parse(values[1]);

        // set boolean for zero motion detection
        isZeroMotion = (opcode == (int)Opcodes.zeromotion);
        
        // if opcode signals calibration event, then grab gyro x-value for Accelerometer Range
        // and grab gyro y-value for Gyroscope Range
        // (this functionality is described in the Curie Sketch)
        if(opcode == (int)Opcodes.calibrate)
        {
            // get range values from Curie (printed only just after calibration)
            // these are needed *before* conversion is done
            CurieAccelerometerRange = rawGyro.x;
            CurieGyroscopeRange = rawGyro.y;
            // set the rawGyro values to 0 now that we've taken the information
            // this makes the operations later on in this function which use the 
            // raw gyro values not produce incorrect results
            rawGyro.x = 0.0f;
            rawGyro.y = 0.0f;
        }

        // convert raw values into G's
        convAcc.Set(
            convertRawAcceleration(rawAcc.x),
            convertRawAcceleration(rawAcc.y),
            convertRawAcceleration(rawAcc.z)
            );
        // convert raw values into radians/time
        convGyro.Set(
            convertRawGyroRad(rawGyro.x),
            convertRawGyroRad(rawGyro.y),
            convertRawGyroRad(rawGyro.z)
            );

        if(opcode == (int)Opcodes.calibrate)
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
        if (previousTime > timestamp)
        {
            // the timer has overflowed back to zero 
            // (unsigned 32-bit value overflows at 4294967295, which is about 70 minutes
            // since the timer is in microseconds)
            float tmp = previousTime;
            tmp -= uint.MaxValue;
            dt = timestamp - tmp;
        }
        else
        {
            dt = timestamp - previousTime;
        }
        // set previous time now that the dt has been calculated
        previousTime = timestamp;
        // change dt to seconds from microseconds
        dt = dt / 1000000.0f;

        // do Kalman filtering for rotation
        kalmanRot.x = kalmanX.getAngle(estRot.x, convGyro.x, dt);
        kalmanRot.y = kalmanY.getAngle(estRot.y, convGyro.y, dt);
        //kalmanRot.z = kalmanZ.getAngle(estRot.z, convGyro.z, dt);
        kalmanRot.z = 0.0f;

        // correct angle according to board's orientation
        //kalmanCorrectedRot = Quaternion.Euler(angleCorrection(getOrientation(rawAcc), kalmanRot));
        kalmanCorrectedRot = Quaternion.Euler(kalmanRot);

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

    // Don't forget to run a coroutine to initialize calibration
    // without coroutine there is no easy way to verify that the
    // user has oriented the Curie before calibration
    // **Calibration should only need to be run once per Curie unit
    public bool Calibrate()
    {
        // start calbration
        serial.Write("c");
        // Curie will block until zero motion is detected, and then calibrate.
        int loops = 0;
        string statement = serial.ReadTo(";");
        string opcode = statement[0] + "";
        // Wait until an opcode of "0" is recieved (this is the calibration opcode)
        // The opcode is a 1-byte
        while((int.Parse(opcode) != (int)Opcodes.calibrate) && (loops < 1000000))
        {
            // read next statement from Curie
            statement = serial.ReadTo(";");
            opcode = statement[0] + "";
            loops += 1;
        }
        if (int.Parse(opcode) != (int)Opcodes.calibrate)
        {
            return false;
        }
        else
        {
            UpdateValues(statement);
            return true;
        }
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
            // flat is (90,90,?)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, 90.0f)) + Mathf.Abs(diffEulerAngles(rot.y, 90.0f));
            info += ("Difference of " + rot + " from (90,90,?): " + modifier + "\n");
        }
        else if (orientation == (int)Orientations.FaceDown)
        {
            // flat is (-90,-90,?)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, -90.0f)) + Mathf.Abs(diffEulerAngles(rot.y, -90.0f));
            info += ("Difference of " + rot + " from (-90,-90,?): " + modifier + "\n");
        }
        else if (orientation == (int)Orientations.DigitalUp)
        {
            // flat is (0,?,90)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, 0.0f)) + Mathf.Abs(diffEulerAngles(rot.z, 90.0f));
            info += ("Difference of " + rot + " from (0,?,90): " + modifier + "\n");
        }
        else if (orientation == (int)Orientations.AnalogUp)
        {
            // flat is (180,?,-90)
            modifier = Mathf.Abs(diffEulerAngles(rot.x, 180.0f)) + Mathf.Abs(diffEulerAngles(rot.z, -90.0f));
            info += ("Difference of " + rot + " from (180,?,-90): " + modifier + "\n");
        }
        else if(orientation == (int)Orientations.ACDown)
        {
            // flat is (?,0,0)
            modifier = Mathf.Abs(diffEulerAngles(rot.y, 0.0f)) + Mathf.Abs(diffEulerAngles(rot.z, 0.0f));
            info += ("Difference of " + rot + " from (?,0,0): " + modifier + "\n");
        }
        else
        {
            // flat is (?,180,180)
            modifier = Mathf.Abs(diffEulerAngles(rot.y, 180.0f)) + Mathf.Abs(diffEulerAngles(rot.z, 180.0f));
            info += ("Difference of " + rot + " from (?,180,180): " + modifier + "\n");
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
        // "baseRot" holds the value which the inaccurate axis will approach as it becomes
        // less accurate (either the x, y, or z value depending on which is the inaccurate axis).
        //
        Vector3 newRot = rot;
        float baseRot;
        // calculate modifier, which is the difference between the "accurate" angles and
        // either 0, 90, 180, -90, or -180, depending on which value the angle is closest to.
        // **this logic is done in function "diffFromFlat"
        float modifier = diffFromFlat(orientation, rot);            
        if(orientation == (int)Orientations.FaceUp || orientation == (int)Orientations.FaceDown)
        {
            // board face up or face down (z-axis is inaccurate)
            info += ("z-axis is inaccurate\n");
            // find base rotation (nearest angle that is "flat")
            if (rot.z > 90)
            {
                baseRot = 180.0f;
            }
            else if (rot.z < -90)
            {
                baseRot = -180.0f;
            }
            else
            {
                baseRot = 0.0f;
            }
            // if modifier is less than 1, the correction will overshoot the "base rotation"
            // so only do the gradual correction when the modifier is greater than one
            if (modifier >= 1)
            {
                // calculate difference between unmodified rotation and the base rotation
                float diff = diffEulerAngles(rot.z,baseRot);
                float newZ = rot.z - (diff / Mathf.Log(modifier,1.3f)); // reduce magnitude of rotation value
                newRot.z = newZ;
            }
            else
            {
                newRot.z = baseRot;
            }
        }
        else if(orientation == (int)Orientations.DigitalUp || orientation == (int)Orientations.AnalogUp)
        {
            // digital pins up or analog pins up (y-axis is inaccurate)
            info += ("y-axis is inaccurate\n");
            if (rot.y > 90)
            {
                baseRot = 180.0f;
            }
            else if (rot.y < -90)
            {
                baseRot = -180.0f;
            }
            else
            {
                baseRot = 0.0f;
            }
            if (modifier >= 1)
            {
                float diff = diffEulerAngles(rot.y, baseRot);
                float newY = rot.y - (diff / Mathf.Log(modifier,1.3f));
                newRot.y = newY;
            }
            else
            {
                newRot.y = baseRot;
            }
        }
        else if(orientation == (int)Orientations.ACDown || orientation == (int)Orientations.ACUp) 
        {
            // AUX facing down or facing up (x-axis is inaccurate)
            info += ("x-axis is inaccurate\n");
            if (rot.x > 90)
            {
                baseRot = 180.0f;
            }
            else if (rot.x < -90)
            {
                baseRot = -180.0f;
            }
            else
            {
                baseRot = 0.0f;
            }
            if (modifier >= 1)
            {
                float diff = diffEulerAngles(rot.x, baseRot);
                float newX = rot.x - (diff / Mathf.Log(modifier,1.3f));
                newRot.x = newX;
            }
            else
            {
                newRot.x = baseRot;
            }
        }
        return newRot;
    }
}
