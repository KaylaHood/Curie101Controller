using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using System;

public class CurieMovement : MonoBehaviour, IDisposable
{
    private int LineMultiplier = 250;
    public UnityEngine.LineRenderer MasterLine;
    public UnityEngine.LineRenderer Slave1Line;
    public UnityEngine.LineRenderer Slave2Line;
    public UnityEngine.LineRenderer FrankenCurieLine;

    private Rigidbody[] boardRigidBodies;
    private MovementTracker[] movementTrackers;
    private FrankenCurieSerialReader monitor;
    private SerialPort port;
    private int baudRate = 38400;
    private bool masterIsCalibrated = false;
    private bool slave1IsCalibrated = false;
    private bool slave2IsCalibrated = false;
    private bool didStartCoroutineMaster = false;
    private bool didStartCoroutineSlave1 = false;
    private bool didStartCoroutineSlave2 = false;
    private bool setupCompleted = false;
    private float timeCalibrated;

    public UnityEngine.UI.Text calibrationMessage;
    public UnityEngine.UI.Text frankenCurieInfo;
    public UnityEngine.UI.Text frankenCurieAccelText;
    public UnityEngine.UI.Text masterAccelText;
    public UnityEngine.UI.Text slave1AccelText;
    public UnityEngine.UI.Text slave2AccelText;

    public event Action<CurieMovement> OnMasterCalibrationComplete;
    public event Action<CurieMovement> OnMasterCalibrationFailed;
    public event Action<CurieMovement> OnSlave1CalibrationComplete;
    public event Action<CurieMovement> OnSlave1CalibrationFailed;
    public event Action<CurieMovement> OnSlave2CalibrationComplete;
    public event Action<CurieMovement> OnSlave2CalibrationFailed;
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
            if (!masterIsCalibrated && !didStartCoroutineMaster)
            {
                didStartCoroutineMaster = true;
                StartCoroutine(CoroutineCalibrateMaster());
            }
            else if(masterIsCalibrated && !slave1IsCalibrated && !didStartCoroutineSlave1)
            {
                didStartCoroutineSlave1 = true;
                StartCoroutine(CoroutineCalibrateSlave1());
            }
            else if(slave1IsCalibrated && !slave2IsCalibrated && !didStartCoroutineSlave2)
            {
                didStartCoroutineSlave2 = true;
                StartCoroutine(CoroutineCalibrateSlave2());
            }
            else if (masterIsCalibrated && slave1IsCalibrated && slave2IsCalibrated)
            {
                monitor.UpdateValues();
                movementTrackers[0].UpdateValues(monitor.curieData.frankenTranslationalAccel, monitor.curieData.frankenFilteredRotation);
                frankenCurieInfo.text = "Unfiltered Euler Rotation:\n " + monitor.curieData.frankenEstRotation.eulerAngles + 
                    "\nFiltered Euler Rotation:\n " + monitor.curieData.frankenFilteredRotation.eulerAngles +
                    "\nAcceleration Vector With Gravity:\n " + monitor.curieData.frankenAccel +
                    "\nGravity Non-Rotated:\n" + monitor.curieData.frankenAccelAtCalibration +
                    "\nGravity Rotated:\n" + monitor.curieData.frankenGravity +
                    "\nAcceleration Vector W/O Gravity:\n " + monitor.curieData.frankenTranslationalAccel;
                calibrationMessage.text = "Sample Run Time: " + (Time.time - timeCalibrated);
                FrankenCurieLine.SetPosition(1, monitor.curieData.frankenAccel * LineMultiplier);
                MasterLine.SetPosition(1, monitor.curieData.convertedAccels[0] * LineMultiplier);
                Slave1Line.SetPosition(1, monitor.curieData.convertedAccels[1] * LineMultiplier);
                Slave2Line.SetPosition(1, monitor.curieData.convertedAccels[2] * LineMultiplier);
                frankenCurieAccelText.text = "FrankenCurie Accel: " + monitor.curieData.frankenAccel;
                masterAccelText.text = "Master Accel: " + monitor.curieData.convertedAccels[0];
                slave1AccelText.text = "Slave 1 Accel: " + monitor.curieData.convertedAccels[1];
                slave2AccelText.text = "Slave 2 Accel: " + monitor.curieData.convertedAccels[2];
            }
        }
        else
        {
            return;
        }
    }

    public IEnumerator CoroutineCalibrateMaster()
    {
        calibrationMessage.text = "Hold Master flat, the press a.";
        OnMasterCalibrationComplete += (CurieMovement a) => {
            masterIsCalibrated = true;
        };
        OnMasterCalibrationFailed += (CurieMovement a) => {
            calibrationMessage.text = ("Master Calibration failed");
            a.enabled = false;
        };
        while(true)
        {
            if(Input.GetKeyDown("a"))
            {
                CalibrateMaster();
                break;
            }
            yield return null;
        }
    }

    public IEnumerator CoroutineCalibrateSlave1()
    {
        calibrationMessage.text = "Hold Slave 1 flat, the press b.";
        OnSlave1CalibrationComplete += (CurieMovement a) => {
            slave1IsCalibrated = true;
        };
        OnSlave1CalibrationFailed += (CurieMovement a) => {
            calibrationMessage.text = ("Slave 1 Calibration failed");
            a.enabled = false;
        };
        while(true)
        {
            if(Input.GetKeyDown("b"))
            {
                CalibrateSlave1();
                break;
            }
            yield return null;
        }
    }

    public IEnumerator CoroutineCalibrateSlave2()
    {
        calibrationMessage.text = "Hold Slave 2 flat, the press c.";
        OnSlave2CalibrationComplete += (CurieMovement a) => {
            slave2IsCalibrated = true;
            timeCalibrated = Time.time;
        };
        OnSlave2CalibrationFailed += (CurieMovement a) => {
            calibrationMessage.text = ("Master Calibration failed");
            a.enabled = false;
        };
        while(true)
        {
            if(Input.GetKeyDown("c"))
            {
                CalibrateSlave2();
                break;
            }
            yield return null;
        }
    }

    public void CalibrateMaster()
    {
        if (monitor.CalibrateMaster())
        {
            OnMasterCalibrationComplete(this);
        }
        else
        {
            OnMasterCalibrationFailed(this);
        }
    }

    public void CalibrateSlave1()
    {
        if (monitor.CalibrateSlave1())
        {
            OnSlave1CalibrationComplete(this);
        }
        else
        {
            OnSlave1CalibrationFailed(this);
        }
    }

    public void CalibrateSlave2()
    {
        if (monitor.CalibrateSlave2())
        {
            OnSlave2CalibrationComplete(this);
        }
        else
        {
            OnSlave2CalibrationFailed(this);
        }
    }

    public void Dispose()
    {
        monitor.Disconnect();
        port.Close();
    }
}
