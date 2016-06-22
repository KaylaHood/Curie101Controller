using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

//Create a new SerialPort property in your class.

class CurieSerialReader
{
    //private string logFileName = "logfile.txt";
    //private System.IO.FileStream logFile;
    private SerialPort serial;
    private enum opcodes { calibrate, normal, zeromotion };
    private float CurieAccelerometerRange;
    private float CurieGyroscopeRange;
    private long previousTime;
    private Vector3 estRotOffset;
    private Vector3 originalGravity;
    public Vector3 acc;
    public Vector3 rot;
    public Vector3 gravity;
    public bool isZeroMotion;
    // Single-Dimensional Kalman Filters
    Kalman kalmanX;
    Kalman kalmanY;
    Kalman kalmanZ;

    public CurieSerialReader(string port)
    {
        // Setup your port object.
        serial = new SerialPort(
        port, 9600, Parity.None, 8, StopBits.One
        );
        // Setup your acceleration and rotation vectors
        acc = Vector3.zero;
        rot = Vector3.zero;
  
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

        // set up log file
        //logFile = System.IO.File.Open(logFileName, System.IO.FileMode.OpenOrCreate, System.IO.FileAccess.Write);
    }
    public CurieSerialReader(SerialPort port)
    {
        // Setup your port object.
        serial = port;
        // Setup your acceleration and rotation vectors
        acc = Vector3.zero;
        rot = Vector3.zero;

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

        // set up log file
        //logFile = System.IO.File.Open(logFileName, System.IO.FileMode.OpenOrCreate, System.IO.FileAccess.Write);
    }

    public void UpdateValues(string statement = "")
    {
        // You may pass-in a pre-read statement, for special uses.
        // For example, this code uses this argument for passing in the
        // calibration statement after requesting calibration.
        //
        // use these later for converted acc and gyro values
        Vector3 convAcc = Vector3.zero;
        Vector3 convGyro = Vector3.zero;
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
        isZeroMotion = (opcode == (int)opcodes.zeromotion);
        
        // if opcode signals calibration event, then grab gyro x-value for Accelerometer Range
        // and grab gyro y-value for Gyroscope Range
        // (this functionality is described in the Curie Sketch)
        if(opcode == (int)opcodes.calibrate)
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
        // convert raw values into degrees/time
        convGyro.Set(
            convertRawGyro(rawGyro.x),
            convertRawGyro(rawGyro.y),
            convertRawGyro(rawGyro.z)
            );

        if(opcode == (int)opcodes.calibrate)
        {
            // set gravity vector (after conversion)
            // this is done because whatever forces are felt by the curie
            // at calibration (when it is flat and stationary) will
            // coorespond to the force of gravity in its frame of reference
            originalGravity = convAcc;
            Debug.Log("accRange: " + CurieAccelerometerRange + "\ngyroRange: " + CurieGyroscopeRange + "\noriginalGravity: " + originalGravity);
        }

        // estimate rotation angles based on accelerometer values
        // roll = x, pitch = y, yaw = z
        Vector3 estRot = Vector3.zero;
        estRot.x = Mathf.Atan2(rawAcc.y, rawAcc.z) * Mathf.Rad2Deg;
        estRot.y = Mathf.Atan2(rawAcc.x, rawAcc.z) * Mathf.Rad2Deg;
        estRot.z = Mathf.Atan2(rawAcc.y, rawAcc.x) * Mathf.Rad2Deg;

        if(opcode == (int)opcodes.calibrate)
        {
            // set base estRot offset
            estRotOffset = estRot;
        }
        // subtract offset from estRot
        estRot -= estRotOffset;

        // set Kalman filters for both corrected and non-corrected rotations
        kalmanX.setAngle(estRot.x);
        kalmanY.setAngle(estRot.y);
        kalmanZ.setAngle(estRot.z);

        // calculate delta time from last update
        long dt;
        if (previousTime > timestamp)
        {
            // the timer has overflowed back to zero 
            // (unsigned 32-bit value overflows at 4294967295, which is about 70 
            // minutes when the timer is in microseconds)
            long tmp = previousTime;
            tmp -= uint.MaxValue;
            dt = timestamp - tmp;
        }
        else
        {
            dt = timestamp;
            dt -= previousTime;
        }
        // set previous time now that the dt has been calculated
        previousTime = timestamp;

        // do Kalman filtering for rotation
        rot.x = kalmanX.getAngle(estRot.x, convGyro.x, dt);
        rot.y = kalmanY.getAngle(estRot.y, convGyro.y, dt);
        rot.z = kalmanZ.getAngle(estRot.z, convGyro.z, dt);

        // use modified version of rotation vector to align gravity with
        // our particular frame of reference.
        Vector3 modRot = new Vector3(-rot.x, rot.y, rot.z);
        gravity = Quaternion.Euler(modRot) * originalGravity;

        // calculate translational acceleration (remove influence of gravity)
        // set acc to be the translational acceleration
        acc = convAcc - gravity;
        Debug.Log("acceleration before gravity is subtracted: " + convAcc + "\nacceleration after gravity is subtracted: " + acc);
        Debug.Log("estimated rotation: " + estRot + "\nfiltered rotation: " + rot);
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
        while((int.Parse(opcode) != (int)opcodes.calibrate) && (loops < 1000000))
        {
            // read next statement from Curie
            statement = serial.ReadTo(";");
            opcode = statement[0] + "";
            loops += 1;
        }
        if (int.Parse(opcode) != (int)opcodes.calibrate)
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

    float convertRawGyro(float gRaw) {
        // since we are using degrees/seconds
        // -(max range) maps to a raw value of -32768
        // +(max range) maps to a raw value of 32767

        float g = (gRaw * CurieGyroscopeRange) / 32768.0f;

        return g;
    }

    int getOrientation(Vector3 rawAcc)
    {
        // 0 = board face up
        // 1 = board face down
        // 2 = digital pins up
        // 3 = analog pins up
        // 4 = AUX facing down
        // 5 = AUX facing up
        int orientation = 0;
        if(Mathf.Abs(rawAcc.z) > Mathf.Abs(rawAcc.y) && Mathf.Abs(rawAcc.z) > Mathf.Abs(rawAcc.x))
        {
            if (rawAcc.z > 0)
            {
                orientation = 0;
            }
            else
            {
                orientation = 1;
            }
        } 
        else if(Mathf.Abs(rawAcc.y) > Mathf.Abs(rawAcc.x) && Mathf.Abs(rawAcc.y) > Mathf.Abs(rawAcc.z))
        {
            if (rawAcc.z > 0)
            {
                orientation = 2;
            }
            else
            {
                orientation = 3;
            }
        } 
        else if(Mathf.Abs(rawAcc.x) > Mathf.Abs(rawAcc.y) && Mathf.Abs(rawAcc.x) > Mathf.Abs(rawAcc.z))
        {
            if (rawAcc.z > 0)
            {
                orientation = 4;
            }
            else
            {
                orientation = 5;
            }

        } 
        return orientation;
    }

    Vector3 angleCorrection(int orientation, Vector3 rot, Vector3 baseRot)
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
        Debug.Log("rotation (angleCorrection):\n" + rot);
        if(orientation == 0 || orientation == 1)
        {
            // board face up or face down (z-axis is inaccurate)
            Debug.Log("z-axis is inaccurate");
            // modifier is the hypotenuse length between the two "accurate" angles
            // max rotation magnitude is 180, so the modifier will be between 0 and 255
            float modifier = Mathf.Sqrt((rot.y * rot.y) + (rot.x * rot.x));
            Debug.Log("modifier: " + modifier);
            if (modifier >= 1)
            {
                // calculate difference between unmodified rotation and the base rotation
                // add 180 degrees to ensure neither are negative
                float diff = (rot.z + 180.0f) - (baseRot.z + 180.0f);
                if(diff > 180)
                {
                    // since the angles roll over from 180 degrees to -180 degrees, if the difference
                    // is greater than 180, I know I should find the "opposite" difference.
                    // (technically, now that I've added 180 degrees to each, the roll over is from 360 to 0)
                    diff = -(360 - diff);
                }
                float newZ = rot.z - ((1 - (modifier / 255.0f)) * diff); // reduce magnitude of z-rotation value
                newRot.z = newZ;
            }
            else
            {
                newRot.z = baseRot.z;
            }
        }
        else if(orientation == 2 || orientation == 3)
        {
            // digital pins up or analog pins up (y-axis is inaccurate)
            Debug.Log("y-axis is inaccurate");
            float modifier = Mathf.Sqrt((rot.z * rot.z) + (rot.x * rot.x));
            Debug.Log("modifier: " + modifier);
            if (modifier >= 1)
            {
                float diff = (rot.y + 180.0f) - (baseRot.y + 180.0f);
                if(diff > 180)
                {
                    diff = -(360 - diff);
                }
                float newY = rot.y - ((1 - (modifier / 255.0f)) * diff); // reduce magnitude of y-rotation value
                newRot.y = newY;
            }
            else
            {
                newRot.y = baseRot.y;
            }
        }
        else if(orientation == 4 || orientation == 5) 
        {
            // AUX facing down or facing up (x-axis is inaccurate)
            Debug.Log("x-axis is inaccurate");
            float modifier = Mathf.Sqrt((rot.z * rot.z) + (rot.y * rot.y));
            Debug.Log("modifier: " + modifier);
            if (modifier >= 1)
            {
                float diff = (rot.x + 180.0f) - (baseRot.x + 180.0f);
                if(diff > 180)
                {
                    diff = -(360 - diff);
                }
                float newX = rot.x - ((1 - (modifier / 255.0f)) * diff); // reduce magnitude of x-rotation value
                newRot.x = newX;
            }
            else
            {
                newRot.x = baseRot.x;
            }
        }
        Debug.Log("new rotation (angleCorrection):\n" + newRot);
        return newRot;
    }
}
