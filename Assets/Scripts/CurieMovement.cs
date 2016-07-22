using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

public class CurieMovement : MonoBehaviour, IDisposable
{

    // contains rigidbody components of all board models
    private Rigidbody[] boardRBs;
    // all movement trackers for the objects controlled by this Curie
    private MovementTracker[] movementTrackers;
    // class for managing Curie data
    private FrankenCurieSerialReader monitor;
    // serial port for communicating with the Curie over a USB connection
    private SerialPort port;
    private int baudRate = 38400;
    // boolean values for calibration
    private bool curieIsCalibrated = false;
    private bool didStartCoroutine = false;
    // boolean value to block updates until setup is finished
    private bool setupCompleted = false;

    // UI text components for demo (not vital to operation)
    public UnityEngine.UI.Text trackingInfo;

    // Action events for calibration coroutine
    public event Action<CurieMovement> OnCalibrationComplete;
    public event Action<CurieMovement> OnCalibrationFailed;
    // Use this for initialization
    void Start()
    {
        port = new SerialPort(
            SerialPort.GetPortNames()[0], baudRate, Parity.None, 8, StopBits.One
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

        monitor = new FrankenCurieSerialReader(port);
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
                //for (int i = 0; i < movementTrackers.Length; i++)
                //{
                //    movementTrackers[i].UpdateValues(monitor.finalAccel, monitor.finalRot);
                //}
                movementTrackers[0].UpdateValues(monitor.curieData.frankenAccel, monitor.curieData.frankenFilteredRotation);
                movementTrackers[1].UpdateValues(monitor.curieData.convertedAccelArray[0], monitor.curieData.filteredRotationArray[0]);
                movementTrackers[2].UpdateValues(monitor.curieData.slave1RotatedAccel, monitor.curieData.filteredRotationArray[1]);
                movementTrackers[3].UpdateValues(monitor.curieData.slave2RotatedAccel, monitor.curieData.filteredRotationArray[2]);
                trackingInfo.text += "\nOverall Update Time: " + Time.time;
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
