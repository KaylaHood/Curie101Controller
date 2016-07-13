using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System;

public class MovementTracker
{
    // myGameObject is an empty game object that is a parent to
    // all of the board model objects
    Rigidbody rigidbody;

    // Use this for initialization
    public MovementTracker(Rigidbody rb)
    {
        rigidbody = rb;
    }

    // Update is called once per frame
    public void UpdateValues(Vector3 acc, Quaternion rot)
    {
        rigidbody.MoveRotation(rot);
        //rigidbody.AddForce(acc, ForceMode.Impulse);
    }
}
