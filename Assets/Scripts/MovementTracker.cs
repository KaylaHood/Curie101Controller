using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System;

public class MovementTracker
{
    CurieSerialReader monitor;
    GameObject myGameObject;
    Vector3 acc;
    Quaternion rot;

    // Use this for initialization
    public MovementTracker(GameObject g)
    {
        myGameObject = g;
        monitor = new CurieSerialReader(SerialPort.GetPortNames()[0]);
        Assert.AreNotEqual(monitor, null);
        //initialize acceleration and rotation vectors
        acc = Vector3.zero;
        rot = Quaternion.identity;
    }
    public MovementTracker(GameObject g, SerialPort port)
    {
        myGameObject = g;
        monitor = new CurieSerialReader(port);
        Assert.AreNotEqual(monitor, null);
        //initialize acceleration and rotation vectors
        acc = Vector3.zero;
        rot = Quaternion.identity;
    }

    // Update is called once per frame
    public void UpdateValues()
    {
        // update Serial Monitor, put new values from Curie into Serial Monitor
        monitor.UpdateValues();
        // check for zero motion signal, if not then do movement operation
        //if (!monitor.isZeroMotion)
        //{
            // get acceleration from Curie
            acc = monitor.acc;
            // round raw acceleration to 0.0 if abs. val. is < .1
            acc.Set
                (
                Utilities.roundf(acc.x),
                Utilities.roundf(acc.y),
                Utilities.roundf(acc.z)
                );
            // multiply by gravity (converting G's -> Newtons)
            // don't multiply by mass because "AddRelativeForce" does that
            acc *= 9.8f;
            // wake up the object by force, in case it refuses to wake up otherwise
            (myGameObject.GetComponent<Rigidbody>()).WakeUp();
            // apply acceleration to the rigidbody component of the object
            // since the curie reports only relative acceleration,
            // we must apply a relative force to the rigidbody.
            (myGameObject.GetComponent<Rigidbody>()).AddRelativeForce(acc,ForceMode.Impulse);
            // update rotation
            rot = Quaternion.Euler(monitor.rot);
            // send new rotation to the rigidbody component of the object
            // by using "MoveRotation," the physics engine should
            // calculate a smooth rotation transition.
            (myGameObject.GetComponent<Rigidbody>()).MoveRotation(rot);
        //}
        //else
        //{
            // zero motion is detected
            // command rigidbody component to sleep
            // this should make the object stop moving
            //(myGameObject.GetComponent<Rigidbody>()).Sleep();
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
