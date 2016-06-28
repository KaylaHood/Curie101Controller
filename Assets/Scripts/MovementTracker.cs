using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System;

public class MovementTracker
{
    public CurieSerialReader monitor;
    // myGameObject is an empty game object that is a parent to
    // all of the board model objects
    GameObject myGameObject;
    // contains rigidbody components of all board models
    Rigidbody[] boardRB;

    // Use this for initialization
    public MovementTracker(GameObject g)
    {
        myGameObject = g;
        monitor = new CurieSerialReader(SerialPort.GetPortNames()[0]);
        Assert.AreNotEqual(monitor, null);
        boardRB = myGameObject.GetComponentsInChildren<Rigidbody>();
    }
    public MovementTracker(GameObject g, SerialPort port)
    {
        myGameObject = g;
        monitor = new CurieSerialReader(port);
        Assert.AreNotEqual(monitor, null);
        boardRB = myGameObject.GetComponentsInChildren<Rigidbody>();
    }

    // Update is called once per frame
    public void UpdateValues()
    {
        // update Serial Monitor, put new values from Curie into Serial Monitor
        monitor.UpdateValues();
        // check for zero motion signal, if not then do movement operation
        //if (!monitor.isZeroMotion)
        //{
            // wake up the objects by force, in case they refuse to wake up otherwise
            foreach (Rigidbody rb in boardRB)
            {
                rb.WakeUp();
            }
            // apply acceleration to the rigidbody component of the object
            // since the curie reports only relative acceleration,
            // we must apply a relative force to the rigidbody.
            //boardRB[0].AddRelativeForce((monitor.acc * 9.8f),ForceMode.Impulse);
            //boardRB[1].AddRelativeForce((monitor.acc * 9.8f),ForceMode.Impulse);
            // send new rotation to the rigidbody component of the object
            // by using "MoveRotation," the physics engine should
            // calculate a smooth rotation transition. (if "Is Kinematic" is true)
            boardRB[0].MoveRotation(monitor.madgwickRot);
            boardRB[1].MoveRotation(monitor.kalmanCorrectedRot);
        //}
        //else
        //{
            // --zero motion is detected--
            // command all rigidbody components to sleep
            //foreach (Rigidbody rb in boardRB)
            //{
            //  rb.Sleep();
            //}
        //}
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

    public event Action<MovementTracker> OnCalibrationComplete;
    public event Action<MovementTracker> OnCalibrationFailed;

}
