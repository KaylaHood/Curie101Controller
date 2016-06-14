using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System.Threading;

//Create a new SerialPort property in your class.

class SerialReader
{
    private SerialPort serial;
    public Vector3 acc;
    public Vector3 rot;
    public bool isZeroMotion;

    public SerialReader(string port)
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
    }
    public SerialReader(SerialPort port)
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
    }

    public void UpdateValues()
    {
        // Send a command to your device.
        serial.Write("u");

        // Read Responses from the serial port.
        // blocks until a line is received.
        // accelerometer prints first,
        // gyroscope prints second on a new line.
        var accmeter = serial.ReadLine();
        if (accmeter == "Zero Motion")
        {
            //received zero motion notification
            isZeroMotion = true;
        }
        else
        {
            var gyro = serial.ReadLine();

            // Parse the values from the response.
            if (accmeter != string.Empty)
            {
                string[] axyz = accmeter.Split(',');

                acc.Set
                    (
                    float.Parse(axyz[0]),
                    float.Parse(axyz[1]),
                    float.Parse(axyz[2])
                    );
                isZeroMotion = false;
                // helpful print statement for detected acceleration
                Debug.Log("Plain acceleration in Gs:");
                Debug.Log(acc);
            }
            if (gyro != string.Empty)
            {
                string[] gxyz = gyro.Split(',');

                rot.Set
                    (
                    float.Parse(gxyz[0]),
                    float.Parse(gxyz[1]),
                    float.Parse(gxyz[2])
                    );
                Debug.Log("Plain rotation in degrees:");
                Debug.Log(rot);
            }
        }
    }

    // Don't forget to run a coroutine to initialize calibration
    // without coroutine there is no easy way to verify that the
    // user has oriented the Curie before calibration
    // **Calibration only needs to be run once per Curie unit
    public bool Calibrate()
    {
        var msg = "";
        // start calbration
        for (int i = 0; i < 100; i++)
        {
            serial.Write("c");
            msg = serial.ReadLine();
            // check for successful calibration message 
            if (msg == "calibrated")
            {
                Debug.Log("Curie's accelerometer and gyroscope have been calibrated");
                return true;
            }
            else if (msg == "cannot calibrate, motion detected")
            {
                Debug.Log("Curie did not calibrate because motion was detected");
                continue;
            }
        }
        Debug.Log("Curie did not calibrate");
        return false;
    }

}
