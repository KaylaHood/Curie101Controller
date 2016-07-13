using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

public class CurieMovement : MonoBehaviour, IDisposable
{

    // contains rigidbody components of all board models
    Rigidbody[] boardRBs;
    // all movement trackers for the objects controlled by this Curie
    MovementTracker[] movementTrackers;
    // class for managing Curie data
    CurieSerialReader monitor;
    // serial port for communicating with the Curie over a USB connection
    SerialPort port;
    // boolean values for calibration
    bool curieIsCalibrated = false;
    bool didStartCoroutine = false;
    // boolean value to block updates until setup is finished
    bool setupCompleted = false;

    // UI text components for demo (not vital to operation)
    public UnityEngine.UI.Text trackingInfo;

    // Action events for calibration coroutine
    public event Action<CurieMovement> OnCalibrationComplete;
    public event Action<CurieMovement> OnCalibrationFailed;

    // Use this for initialization
    void Start()
    {
        port = new SerialPort(
            SerialPort.GetPortNames()[0], 9600, Parity.None, 8, StopBits.One
        );
        port.Open();

        // get all rigidbody components of objects
        boardRBs = gameObject.GetComponentsInChildren<Rigidbody>();

        // construct movement trackers
        movementTrackers = new MovementTracker[boardRBs.Length];
        for (int i = 0;i < boardRBs.Length;i++)
        {
            movementTrackers[i] = new MovementTracker(boardRBs[i]);
            Assert.AreNotEqual(movementTrackers[i], null);
        }
        // Allow time for objects to be constructed.
        // If this is not done, the coroutine *could* start before the objects are constructed.
        Thread.Sleep(100);

        monitor = new CurieSerialReader(port);
        Assert.AreNotEqual(monitor, null);

        // set UI text component variable of monitor in the movement tracker
        // this allows the movement tracker to update the UI text box with
        // important debug information
        monitor.trackingInfo = trackingInfo;

        // set boolean to indicate that setup has completed
        setupCompleted = true;
    }

    // Update is called once per frame
    void Update()
    {
        if (setupCompleted)
        {
            if (!curieIsCalibrated && !didStartCoroutine)
            {
                didStartCoroutine = true;
                StartCoroutine(CoroutineCalibrate());
            }
            else if (curieIsCalibrated)
            {
                // update Serial Monitor, put new values from Curie into Serial Monitor
                monitor.UpdateValues();
                // update all movement tracking scripts
                for (int i = 0; i < movementTrackers.Length; i++)
                {
                    if (i != 0)
                    {
                        movementTrackers[i].UpdateValues(monitor.acc, monitor.kalmanCorrectedRot);
                    }
                    else
                    {
                        movementTrackers[i].UpdateValues(monitor.acc, monitor.madgwickRot);
                    }
                }
            }
        }
        else
        {
            return;
        }
    }

    public IEnumerator CoroutineCalibrate()
    {
        OnCalibrationComplete += (CurieMovement a) => {
            Debug.Log("Calibration finished sucessfully");
            curieIsCalibrated = true;
        };
        OnCalibrationFailed += (CurieMovement a) => {
            Debug.Log("Calibration failed");
        };
        trackingInfo.text = "Make sure that Curie is held flat for calibration. Hold y when ready.";
        Debug.Log("Make sure that Curie is held flat for calibration. Hold y when ready.");
        while(true)
        {
            if(Input.GetKeyDown("y"))
            {
                trackingInfo.text = "Now Calibrating... hold Curie still";
                Debug.Log("Now Calibrating... hold Curie still");
                Calibrate();
                break;
            }
            yield return null;
        }
    }

    // this just calls the SerialReader's Calibrate() function
    public void Calibrate()
    {
        if (monitor.Calibrate())
        {
            OnCalibrationComplete(this);
        }
        else
        {
            OnCalibrationFailed(this);
        }
    }


    public void Dispose()
    {
        monitor.Disconnect();
        port.Close();
    }
}
