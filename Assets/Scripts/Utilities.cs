using UnityEngine;
using System.Collections;

public class Utilities {
    public static void printCoords3<T>(T x, T y, T z)
    {
        Debug.Log(x + "," + y + "," + z);
    }
    public static float roundf(float x)
    {
        if ((x > 0.0f && x < .1f) || (x < 0.0f && x > -.1f))
        {
            return 0.0f;
        }
        else return x;
    }
}
