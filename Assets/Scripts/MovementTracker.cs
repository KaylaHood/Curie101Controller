using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System;

public class MovementTracker
{
    SerialReader monitor;
    GameObject myGameObject;
    float scalarAcc = 10.0f; // to transform G's to "units traveled per frame squared"
    Vector3 acc;
    Vector3 rot;

    // Use this for initialization
    public MovementTracker(GameObject g)
    {
        myGameObject = g;
        monitor = new SerialReader(SerialPort.GetPortNames()[0]);
        Assert.AreNotEqual(monitor, null);
        //calibrate Curie
        monitor.Calibrate();
        //initialize acceleration and rotation vectors
        acc = Vector3.zero;
        rot = Vector3.zero;
        monitor.UpdateValues();
    }
    public MovementTracker(GameObject g, SerialPort port)
    {
        myGameObject = g;
        monitor = new SerialReader(port);
        Assert.AreNotEqual(monitor, null);
        //calibrate Curie
        monitor.Calibrate();
        //initialize acceleration and rotation vectors
        acc = Vector3.zero;
        rot = Vector3.zero;
        monitor.UpdateValues();
    }

    // Update is called once per frame
    public void UpdateValues()
    {
        // update Serial Monitor, put new values from Curie into Serial Monitor
        monitor.UpdateValues();
        // check for zero motion signal, if not then do movement operation
        if (!monitor.isZeroMotion)
        {
            // get acceleration from Curie
            acc = monitor.acc;
            // round raw acceleration to 0.0 if abs. val. is < .1
            Vector3 modAcc = new Vector3
                (
                Utilities.roundf(acc.x),
                Utilities.roundf(acc.y),
                Utilities.roundf(acc.z)
                );
            // multiply by scalar constant
            modAcc *= scalarAcc;
            // helpful print statement for mod. acceleration
            Debug.Log("modifiers:");
            Debug.Log(modAcc);
            // wake up the object by force, in case it refuses to wake up otherwise
            (myGameObject.GetComponent<Rigidbody>()).WakeUp();
            // apply acceleration to the rigidbody component of the object
            // since the curie reports only relative acceleration,
            // we must apply a relative force to the rigidbody.
            (myGameObject.GetComponent<Rigidbody>()).AddRelativeForce(modAcc,ForceMode.Acceleration);
            // update rotation
            rot = monitor.rot;
            // helpful print statement for new rotation
            Debug.Log("new rotation angles (degrees):");
            Debug.Log(rot);
            // send new rotation to the rigidbody component of the object
            // by using "MoveRotation," the physics engine should
            // calculate a smooth rotation transition.
            (myGameObject.GetComponent<Rigidbody>()).MoveRotation(Quaternion.Euler(rot));
        }
        else
        {
            // zero motion is detected
            // command rigidbody component to sleep
            // this should make the object stop moving
            (myGameObject.GetComponent<Rigidbody>()).Sleep();
            // helpful print statement
            Debug.Log("Object stationary, zero motion detected.");
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

    public event Action<MovementTracker> OnCalibrationComplete;
    public event Action<MovementTracker> OnCalibrationFailed;

}
