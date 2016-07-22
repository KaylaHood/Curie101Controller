using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using System;

// -----------------------------------------------------------------------------------------------------------
// This Document is Normalized to 110 Columns Wide
// -----------------------------------------------------------------------------------------------------------

public class FrankenCurieSerialReader
{
    // -------------------------------------------------------------------------------------------------------
    // Private Variable Declarations
    // -------------------------------------------------------------------------------------------------------

    private bool debug = true;

    public SerialPort serial;
    private int baudRate = 38400;
    private int serialMessageSize = 34;

    // -------------------------------------------------------------------------------------------------------
    // Public Variable Declarations
    // -------------------------------------------------------------------------------------------------------

    public CurieDataManager curieData;

    public bool isZeroMotion;

    public UnityEngine.UI.Text trackingInfo = null;
    string info;

    // -------------------------------------------------------------------------------------------------------
    // Interior Class Definitions
    // -------------------------------------------------------------------------------------------------------

    // CurieDataManager Class for easy management of data from Curie

    public class CurieDataManager
    {
        public SerialPort serial;
        private int messageLength;
        private byte[] message;

        // constant values for Opcodes
        public int normal = 0;
        public int zeromotion = 1;
        public int calibration = 2;
        // constant values for Orientations
        public int FaceUp = 0;
        public int FaceDown = 1;
        public int DigitalUp = 2;
        public int AnalogUp = 3;
        public int ACDown = 4;
        public int ACUp = 5;

        public float CurieAccelerometerRange;
        public float CurieGyroscopeRange;
        
        private string calibrationMsg = "c";
        private string beginMsg = "y";
        private string disconnectionMsg = "d";
        public string msgString;
        
        public short opcode;
        public ulong timestamp;
        public float dt;
        public Vector3[] rawAccelArray;
        public Vector3[] convertedAccelArray;
        public Vector3[] originalGravityArray;
        public Vector3[] rotatedGravityArray;
        public Vector3[] translationalAccelArray;
        public Quaternion[] estimatedRotationArray;
        public Quaternion[] filteredRotationArray;
        public Quaternion slave1RotationFromMaster;
        public Quaternion slave2RotationFromMaster;
        public Vector3 slave1RotatedAccel;
        public Vector3 slave2RotatedAccel;
        public Vector3 rawGyro;
        public Vector3 convertedGyro;
        
        public Kalman kalmanX;
        public Kalman kalmanY;
        public Kalman kalmanZ;

        private ulong previousTime;

        private Vector3 frankenOriginalGravity;
        private Vector3 frankenGravity;
        public Vector3 frankenAccel;
        public Vector3 frankenTranslationalAccel;
        public Quaternion frankenEstRotation;
        public Quaternion frankenFilteredRotation;

        private bool messageHasBeenProcessed = false;

        public int timesRead = 0;

        public CurieDataManager(SerialPort s, int msgLen)
        {
            serial = s;
            messageLength = msgLen;
            message = new byte[msgLen];
            kalmanX = new Kalman();
            kalmanY = new Kalman();
            kalmanZ = new Kalman();

            rawAccelArray = new Vector3[3];
            convertedAccelArray = new Vector3[3];
            originalGravityArray = new Vector3[3];
            rotatedGravityArray = new Vector3[3];
            translationalAccelArray = new Vector3[3];
            estimatedRotationArray = new Quaternion[3];
            filteredRotationArray = new Quaternion[3];

            slave1RotationFromMaster = Quaternion.identity;
            slave2RotationFromMaster = Quaternion.identity;
            slave1RotatedAccel = Vector3.zero;
            slave2RotatedAccel = Vector3.zero;

            rawGyro = Vector3.zero;
            convertedGyro = Vector3.zero;

            frankenOriginalGravity = Vector3.zero;
            frankenGravity = Vector3.zero;
            frankenAccel = Vector3.zero;
            frankenTranslationalAccel = Vector3.zero;
            frankenEstRotation = Quaternion.identity;
            frankenFilteredRotation = Quaternion.identity;
        }

        public void RequestCalibration()
        {
            serial.Write(calibrationMsg);
        }
        public void Begin()
        {
            serial.Write(beginMsg);
        }
        public void Disconnect()
        {
            serial.Write(disconnectionMsg);
        }

        public void UpdateData(bool forceRead = false)
        {
            if(messageHasBeenProcessed || forceRead)
            {
                readBytesFromSerial();
                opcode = getOpcode();
                timestamp = getTimestamp();
                rawAccelArray = getAcc();
                rawGyro = getGyro();
            }
        }

        public void UpdateDataAndProcess(bool forceRead = false)
        {
            if(messageHasBeenProcessed || forceRead)
            {
                readBytesFromSerial();
                opcode = getOpcode();
                timestamp = getTimestamp();
                rawAccelArray = getAcc();
                rawGyro = getGyro();
            }
            if (opcode == calibration)
            {
                CurieAccelerometerRange = rawGyro.x;
                CurieGyroscopeRange = rawGyro.y;
            }

            convertedAccelArray[0].Set(
                convertRawAcceleration(rawAccelArray[0].x),
                convertRawAcceleration(rawAccelArray[0].y),
                convertRawAcceleration(rawAccelArray[0].z)
                );
            convertedAccelArray[1].Set(
                convertRawAcceleration(rawAccelArray[1].x),
                convertRawAcceleration(rawAccelArray[1].y),
                convertRawAcceleration(rawAccelArray[1].z)
                );
            convertedAccelArray[2].Set(
                convertRawAcceleration(rawAccelArray[2].x),
                convertRawAcceleration(rawAccelArray[2].y),
                convertRawAcceleration(rawAccelArray[2].z)
                );
            convertedGyro.Set(
                convertRawGyroRad(rawGyro.x),
                convertRawGyroRad(rawGyro.y),
                convertRawGyroRad(rawGyro.z)
                );

            estimatedRotationArray = new Quaternion[] {
                Quaternion.LookRotation(convertedAccelArray[0]),
                Quaternion.LookRotation(convertedAccelArray[1]),
                Quaternion.LookRotation(convertedAccelArray[2])
                };

            slave1RotationFromMaster = Math3d.SubtractRotation(estimatedRotationArray[1],estimatedRotationArray[0]);
            slave2RotationFromMaster = Math3d.SubtractRotation(estimatedRotationArray[2],estimatedRotationArray[0]);

            slave1RotatedAccel = slave1RotationFromMaster * convertedAccelArray[1];
            slave2RotatedAccel = slave2RotationFromMaster * convertedAccelArray[2];
            frankenAccel = (convertedAccelArray[0] + slave1RotatedAccel + slave2RotatedAccel) / 3;

            if(opcode == calibration)
            {
                frankenOriginalGravity = frankenAccel;

                originalGravityArray[0] = convertedAccelArray[0];
                originalGravityArray[1] = slave1RotatedAccel;
                originalGravityArray[2] = slave2RotatedAccel;
            }

            // ---------------------------------------------------------------------------------------------------
            // Calculate Accelerometer-Based Rotation of Franken Curie
            // ---------------------------------------------------------------------------------------------------

            // TODO
            frankenEstRotation = Quaternion.LookRotation(frankenAccel);

            // ---------------------------------------------------------------------------------------------------
            // Calculate Time Difference From Previous Update
            // ---------------------------------------------------------------------------------------------------
            dt = (timestamp - previousTime) / 1000000.0f;
            previousTime = timestamp;

            // ---------------------------------------------------------------------------------------------------
            // Use Kalman Filter to Filter Rotation Value
            // ---------------------------------------------------------------------------------------------------
            Vector3 eulerFrankenRotation = frankenEstRotation.eulerAngles;
            kalmanX.setAngle(eulerFrankenRotation.x);
            kalmanY.setAngle(eulerFrankenRotation.y);
            kalmanZ.setAngle(eulerFrankenRotation.z);

            Vector3 frankenKalmanRot = Vector3.zero;
            frankenKalmanRot.x = kalmanX.getAngle(eulerFrankenRotation.x, convertedGyro.x, dt);
            frankenKalmanRot.y = kalmanY.getAngle(eulerFrankenRotation.y, convertedGyro.y, dt);
            frankenKalmanRot.z = kalmanZ.getAngle(eulerFrankenRotation.z, convertedGyro.z, dt);

            frankenFilteredRotation = Quaternion.Euler(frankenKalmanRot);

            Vector3 eulerMasterRotation = estimatedRotationArray[0].eulerAngles;
            kalmanX.setAngle(eulerMasterRotation.x);
            kalmanY.setAngle(eulerMasterRotation.y);
            kalmanZ.setAngle(eulerMasterRotation.z);

            Vector3 masterKalmanRot = Vector3.zero;
            masterKalmanRot.x = kalmanX.getAngle(eulerMasterRotation.x, convertedGyro.x, dt);
            masterKalmanRot.y = kalmanY.getAngle(eulerMasterRotation.y, convertedGyro.y, dt);
            masterKalmanRot.z = kalmanZ.getAngle(eulerMasterRotation.z, convertedGyro.z, dt);

            filteredRotationArray[0] = Quaternion.Euler(masterKalmanRot);

            Vector3 eulerSlave1Rotation = estimatedRotationArray[1].eulerAngles;
            kalmanX.setAngle(eulerSlave1Rotation.x);
            kalmanY.setAngle(eulerSlave1Rotation.y);
            kalmanZ.setAngle(eulerSlave1Rotation.z);

            Vector3 slave1KalmanRot = Vector3.zero;
            slave1KalmanRot.x = kalmanX.getAngle(eulerSlave1Rotation.x, convertedGyro.x, dt);
            slave1KalmanRot.y = kalmanY.getAngle(eulerSlave1Rotation.y, convertedGyro.y, dt);
            slave1KalmanRot.z = kalmanZ.getAngle(eulerSlave1Rotation.z, convertedGyro.z, dt);

            filteredRotationArray[1] = Quaternion.Euler(slave1KalmanRot);

            Vector3 eulerSlave2Rotation = estimatedRotationArray[2].eulerAngles;
            kalmanX.setAngle(eulerSlave2Rotation.x);
            kalmanY.setAngle(eulerSlave2Rotation.y);
            kalmanZ.setAngle(eulerSlave2Rotation.z);

            Vector3 slave2KalmanRot = Vector3.zero;
            slave2KalmanRot.x = kalmanX.getAngle(eulerSlave2Rotation.x, convertedGyro.x, dt);
            slave2KalmanRot.y = kalmanY.getAngle(eulerSlave2Rotation.y, convertedGyro.y, dt);
            slave2KalmanRot.z = kalmanZ.getAngle(eulerSlave2Rotation.z, convertedGyro.z, dt);

            filteredRotationArray[2] = Quaternion.Euler(slave2KalmanRot);

            // ---------------------------------------------------------------------------------------------------
            // Final Conversions and Setting of Global Variables
            // ---------------------------------------------------------------------------------------------------
            frankenFilteredRotation = Quaternion.Euler(frankenKalmanRot);
            frankenGravity = frankenFilteredRotation * frankenOriginalGravity;
            frankenTranslationalAccel = frankenAccel - frankenGravity;

            rotatedGravityArray[0] = filteredRotationArray[0] * originalGravityArray[0];
            rotatedGravityArray[1] = filteredRotationArray[1] * originalGravityArray[1];
            rotatedGravityArray[2] = filteredRotationArray[2] * originalGravityArray[2];

            translationalAccelArray[0] = convertedAccelArray[0] - rotatedGravityArray[0];
            translationalAccelArray[1] = slave1RotatedAccel - rotatedGravityArray[1];
            translationalAccelArray[2] = slave2RotatedAccel - rotatedGravityArray[2];

            messageHasBeenProcessed = true;
        }

        public void readBytesFromSerial()
        {
            for (int i = 0; i < messageLength; i++)
            {
                message[i] = (byte)serial.ReadByte();
            }
            timesRead += 1;
            messageHasBeenProcessed = false;
        }

        public short getOpcode()
        {
            return BitConverter.ToInt16(message, 0);
        }

        public ulong getTimestamp()
        {
            return BitConverter.ToUInt64(message, 2);
        }

        public Vector3[] getAcc()
        {
            Vector3 A1 = Vector3.zero;
            Vector3 A2 = Vector3.zero;
            Vector3 A3 = Vector3.zero;
            A1.x = (float)BitConverter.ToInt16(message, 10);
            A1.y = (float)BitConverter.ToInt16(message, 12);
            A1.z = (float)BitConverter.ToInt16(message, 14);
            A2.x = (float)BitConverter.ToInt16(message, 16);
            A2.y = (float)BitConverter.ToInt16(message, 18);
            A2.z = (float)BitConverter.ToInt16(message, 20);
            A3.x = (float)BitConverter.ToInt16(message, 22);
            A3.y = (float)BitConverter.ToInt16(message, 24);
            A3.z = (float)BitConverter.ToInt16(message, 26);
            Vector3[] result = { A1, A2, A3 };
            return result;
        }

        public Vector3 getGyro()
        {
            Vector3 G = Vector3.zero;
            G.x = (float)BitConverter.ToInt16(message, 28);
            G.y = (float)BitConverter.ToInt16(message, 30);
            G.z = (float)BitConverter.ToInt16(message, 32);
            return G;
        }

        public string getString()
        {
            return BitConverter.ToString(message);
        }

        float convertRawAcceleration(float aRaw)
        {
            // since we are using Gs :
            // -(max range) maps to a raw value of -32768
            // +(max range) maps to a raw value of 32767

            float a = (aRaw * CurieAccelerometerRange) / 32768.0f;

            return a;
        }

        float convertRawGyroDeg(float gRaw)
        {
            // since we are using degrees/seconds :
            // -(max range) maps to a raw value of -32768
            // +(max range) maps to a raw value of 32767

            float g = (gRaw * CurieGyroscopeRange) / 32768.0f;

            return g;
        }

        float convertRawGyroRad(float gRaw)
        {
            float g = convertRawGyroDeg(gRaw);
            // convert to radians/second
            g *= Mathf.PI;
            g /= 180.0f;
            return g;
        }

        Quaternion getFrankenRot()
        {
            // IDEA FOR IMPROVEMENT -- Use matrices instead of Vector3[] to manipulate estimated rotations
            //                         and to remove inaccurate angles using only matrix operations.
            //
            Quaternion result = Quaternion.identity;

            return result;
        }
    }

    public class BLEMessage
    {
        //~~~~**************~~~~
        //~~~~****_TODO_****~~~~
        //~~~~**************~~~~
        // ********* DONT FORGET TO UPDATE USING COROUTINE TO AVOID LOSS OF DATA *****************************
        public byte[] message;
        short opcode;
        ulong timestamp;
        Vector3 rawAcc;
        Vector3 rawGyro;

        public BLEMessage(byte[] m, short op)
        {
            message = m;
            opcode = op;
            // do work to set other variables
        }
    }

    // -------------------------------------------------------------------------------------------------------
    // CurieSerialReader Class Function Definitions
    // -------------------------------------------------------------------------------------------------------

    public FrankenCurieSerialReader(string port)
    {
        serial = new SerialPort(
        port, baudRate, Parity.None, 8, StopBits.One
        );
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 20000;

        Thread.Sleep(100);

        curieData = new CurieDataManager(serial, serialMessageSize);

        curieData.Begin();
    }
    public FrankenCurieSerialReader(SerialPort port)
    {
        serial = port;
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 20000;

        Thread.Sleep(100);

        curieData = new CurieDataManager(serial, serialMessageSize);

        curieData.Begin();
    }

    public void UpdateValues()
    {
        // ---------------------------------------------------------------------------------------------------
        // Get Values From Serial Port
        // ---------------------------------------------------------------------------------------------------
        curieData.UpdateDataAndProcess();

        isZeroMotion = (curieData.opcode == curieData.zeromotion);

        // ---------------------------------------------------------------------------------------------------
        // Set "Info" String
        // ---------------------------------------------------------------------------------------------------
        info = "Received this statement: " + curieData.msgString + "\n";

        info += ("Values interpreted: " + curieData.opcode + " | " + curieData.timestamp + " | " + 
            curieData.rawAccelArray[0] + " | " + curieData.rawAccelArray[1] + " | " + curieData.rawAccelArray[2] + 
            " | " + curieData.rawGyro + "\n");

        // ---------------------------------------------------------------------------------------------------
        // Calculate Averaged Franken Curie Acceleration
        // ---------------------------------------------------------------------------------------------------

        info += ("timestep: " + curieData.dt + "\nUnity delta time from last update: " + Time.deltaTime);
        trackingInfo.text = info;

        curieData.serial.DiscardInBuffer();
    }

    // **Calibration should only need to be run once per Curie unit
    public bool Calibrate()
    {
        int loops = 0;
        int tries = 0;

        while ((curieData.opcode != curieData.calibration) && (tries < 20))
        {
            if (debug)
            {
                Debug.Log("try: " + tries);
            }
            // ~~~ Request Calibration ~~~
            curieData.serial.DiscardOutBuffer();
            curieData.RequestCalibration();
            while ((curieData.opcode != curieData.calibration) && (loops < 100))
            {
                if (debug)
                {
                    Debug.Log("loop: " + loops);
                    Debug.Log("Not calibration: " + curieData.getString());
                }
                curieData.UpdateData(true);
                loops += 1;
            }
            tries += 1;
            loops = 0;
        }
        if (curieData.opcode != curieData.calibration)
        {
            return false;
        }
        else
        {
            if (debug)
            {
                Debug.Log("Calibration: " + curieData.getString());
            }
            return true;
        }
    }

    public void Disconnect()
    {
        curieData.Disconnect();
    }



}
