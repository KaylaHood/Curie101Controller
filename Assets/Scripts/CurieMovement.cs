using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

public class CurieMovement : MonoBehaviour, IDisposable
{

    MovementTracker movement;
    SerialPort port;
    bool movementIsCalibrated = false;
    bool didStartCoroutine = false;
    // Use this for initialization
    void Start()
    {
        port = new SerialPort(
            SerialPort.GetPortNames()[0], 9600, Parity.None, 8, StopBits.One
        );
        port.Open();
        movement = new MovementTracker(gameObject, port);
        // Allow time for object to be constructed.
        // If this is not done, the coroutine *could* start before the object is constructed.
        Thread.Sleep(100);
        Assert.AreNotEqual(movement, null);
    }

    // Update is called once per frame
    void Update()
    {
        if (!movementIsCalibrated && !didStartCoroutine)
        {
            didStartCoroutine = true;
            Debug.Log("Make sure that Curie is held flat for calibration. Hold y when ready.");
            StartCoroutine(CoroutineCalibrate());
        }
        else if(movementIsCalibrated)
        {
            movement.UpdateValues();
        }
    }

    public IEnumerator CoroutineCalibrate()
    {
        movement.OnCalibrationComplete += (MovementTracker a) => {
            Debug.Log("Calibration finished sucessfully");
            movementIsCalibrated = true;
        };
        movement.OnCalibrationFailed += (MovementTracker a) => {
            Debug.Log("Calibration failed");
        };
        Debug.Log("Make sure that Curie is held flat for calibration. Hold y when ready.");
        while(true)
        {
            if(Input.GetKeyDown("y"))
            {
                Debug.Log("Now Calibrating... hold Curie still");
                movement.Calibrate();
                break;
            }
            yield return null;
        }
    }

    public void Dispose()
    {
        port.Close();
    }
}
