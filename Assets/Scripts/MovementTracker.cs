using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;
using System.IO.Ports;
using System;

public class MovementTracker
{
    private Rigidbody rigidbody;

    public MovementTracker(Rigidbody rb)
    {
        rigidbody = rb;
    }

    public void UpdateValues(Vector3 acc, Quaternion rot)
    {
        rigidbody.rotation = rot;
        //rigidbody.AddForce(acc, ForceMode.Impulse);
    }
}
