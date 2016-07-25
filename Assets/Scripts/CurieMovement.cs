using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

public class CurieMovement : MonoBehaviour, IDisposable
{

    // contains rigidbody components of all board models
    private Rigidbody[] boardRigidBodies;
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
    private float timeCalibrated;

    // UI text components for demo (not vital to operation)
    public UnityEngine.UI.Text calibrationMessage;
    public UnityEngine.UI.Text frankenCurieInfo;
    public UnityEngine.UI.Text masterInfo;
    public UnityEngine.UI.Text slave1Info;
    public UnityEngine.UI.Text slave2Info;

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
        boardRigidBodies = gameObject.GetComponentsInChildren<Rigidbody>();

        // construct movement trackers
        movementTrackers = new MovementTracker[boardRigidBodies.Length];
        for (int i = 0;i < boardRigidBodies.Length;i++)
        {
            movementTrackers[i] = new MovementTracker(boardRigidBodies[i]);
            Assert.AreNotEqual(movementTrackers[i], null);
        }
        // Allow time for objects to be constructed.
        // If this is not done, the coroutine *could* start before the objects are constructed.
        Thread.Sleep(100);

        monitor = new FrankenCurieSerialReader(port);
        Assert.AreNotEqual(monitor, null);

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
                movementTrackers[1].UpdateValues(monitor.curieData.translationalAccels[0], monitor.curieData.filteredRotations[0]);
                movementTrackers[2].UpdateValues(monitor.curieData.translationalAccels[1], monitor.curieData.filteredRotations[1]);
                movementTrackers[3].UpdateValues(monitor.curieData.translationalAccels[2], monitor.curieData.filteredRotations[2]);
                frankenCurieInfo.text = "Unfiltered Euler Rotation:\n " + monitor.curieData.frankenEstRotation.eulerAngles + 
                    "\nFiltered Euler Rotation:\n " + monitor.curieData.frankenFilteredRotation.eulerAngles +
                    "\nAcceleration Vector With Gravity:\n " + monitor.curieData.frankenAccel +
                    "\nGravity Non-Rotated:\n" + monitor.curieData.frankenOriginalGravity +
                    "\nGravity Rotated:\n" + monitor.curieData.frankenGravity +
                    "\nAcceleration Vector W/O Gravity:\n " + monitor.curieData.frankenTranslationalAccel;
                masterInfo.text = "Unfiltered Euler Rotation:\n " + monitor.curieData.estRotations[0].eulerAngles + 
                    "\nFiltered Euler Rotation:\n " + monitor.curieData.filteredRotations[0].eulerAngles +
                    "\nAcceleration Vector:\n " + monitor.curieData.convertedAccels[0] +
                    "\nGravity Non-Rotated:\n" + monitor.curieData.convertedAccelsAtCalibration[0] +
                    "\nGravity Rotated:\n" + monitor.curieData.rotatedGravities[0] +
                    "\nAcceleration Vector W/O Gravity:\n " + monitor.curieData.translationalAccels[0];
                slave1Info.text = "Unfiltered Euler Rotation:\n " + monitor.curieData.estRotations[1].eulerAngles +
                    "\nFiltered Euler Rotation:\n " + monitor.curieData.filteredRotations[1].eulerAngles +
                    "\nNon-Rotated Acceleration Vector:\n" + monitor.curieData.convertedAccels[1] +
                    "\nRotation from Master:\n" + monitor.curieData.slave1RotationFromMaster.eulerAngles +
                    "\nRotated Acceleration Vector:\n " + monitor.curieData.slave1RotatedAccel +
                    "\nGravity Non-Rotated:\n" + monitor.curieData.convertedAccelsAtCalibration[1] +
                    "\nGravity Rotated:\n" + monitor.curieData.rotatedGravities[1] +
                    "\nAcceleration Vector W/O Gravity:\n " + monitor.curieData.translationalAccels[1];
                slave2Info.text = "Unfiltered Euler Rotation:\n " + monitor.curieData.estRotations[2].eulerAngles + 
                    "\nFiltered Euler Rotation:\n " + monitor.curieData.filteredRotations[2].eulerAngles +
                    "\nNon-Rotated Acceleration Vector:\n" + monitor.curieData.convertedAccels[2] +
                    "\nRotation from Master:\n" + monitor.curieData.slave2RotationFromMaster.eulerAngles +
                    "\nRotated Acceleration Vector:\n " + monitor.curieData.slave2RotatedAccel +
                    "\nGravity Non-Rotated:\n" + monitor.curieData.convertedAccelsAtCalibration[2] +
                    "\nGravity Rotated:\n" + monitor.curieData.rotatedGravities[2] +
                    "\nAcceleration Vector W/O Gravity:\n " + monitor.curieData.translationalAccels[2];
                calibrationMessage.text = "Sample Run Time: " + (Time.time - timeCalibrated);
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
            timeCalibrated = Time.time;
            calibrationMessage.text = "Sample Run Time: " + (Time.time - timeCalibrated);
            curieIsCalibrated = true;
        };
        OnCalibrationFailed += (CurieMovement a) => {
            Debug.Log("Calibration failed");
        };
        calibrationMessage.text = "Make sure that Curie is held flat for calibration. Hold y when ready.";
        Debug.Log("Make sure that Curie is held flat for calibration. Hold y when ready.");
        while(true)
        {
            if(Input.GetKeyDown("y"))
            {
                calibrationMessage.text = "Now Calibrating... hold Curie still";
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
