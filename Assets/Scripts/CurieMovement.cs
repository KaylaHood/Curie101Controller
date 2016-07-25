using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

public class CurieMovement : MonoBehaviour, IDisposable
{

    private Rigidbody[] boardRigidBodies;
    private MovementTracker[] movementTrackers;
    private FrankenCurieSerialReader monitor;
    private SerialPort port;
    private int baudRate = 38400;
    private bool curieIsCalibrated = false;
    private bool didStartCoroutine = false;
    private bool setupCompleted = false;
    private float timeCalibrated;

    public UnityEngine.UI.Text calibrationMessage;
    public UnityEngine.UI.Text frankenCurieInfo;
    public UnityEngine.UI.Text masterInfo;
    public UnityEngine.UI.Text slave1Info;
    public UnityEngine.UI.Text slave2Info;

    public event Action<CurieMovement> OnCalibrationComplete;
    public event Action<CurieMovement> OnCalibrationFailed;
    void Start()
    {
        port = new SerialPort(
            SerialPort.GetPortNames()[0], baudRate, Parity.None, 8, StopBits.One
        );
        port.Open();

        boardRigidBodies = gameObject.GetComponentsInChildren<Rigidbody>();

        movementTrackers = new MovementTracker[boardRigidBodies.Length];
        for (int i = 0;i < boardRigidBodies.Length;i++)
        {
            movementTrackers[i] = new MovementTracker(boardRigidBodies[i]);
            Assert.AreNotEqual(movementTrackers[i], null);
        }
        Thread.Sleep(100);

        monitor = new FrankenCurieSerialReader(port);
        Assert.AreNotEqual(monitor, null);

        setupCompleted = true;
    }

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
                monitor.UpdateValues();
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
