using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using System;

public class FrankenCurieSerialReader
{
    private bool debug = true;

    public SerialPort serial;
    private int baudRate = 38400;
    private int serialMessageSize = 46;

    public CurieDataManager curieData;

    public bool isZeroMotion;

    public class CurieDataManager
    {
        public SerialPort serial;
        private int messageLength;
        private byte[] message;

        public int normal = 0;
        public int zeroMotion = 1;
        public int calibration = 2;
        public int slave1Calibration = 3;
        public int slave2Calibration = 4;
        public int FaceUp = 0;
        public int FaceDown = 1;
        public int DigitalUp = 2;
        public int AnalogUp = 3;
        public int ACDown = 4;
        public int ACUp = 5;

        public float CurieAccelerometerRange;
        public float CurieGyroscopeRange;
        
        private string calibrationMsg = "a";
        private string slave1CalibrationMsg = "b";
        private string slave2CalibrationMsg = "c";
        private string beginMsg = "y";
        private string disconnectionMsg = "d";
        public string msgString;
        
        public short opcode;
        public ulong timestamp;
        public float dt;
        public Vector3[] rawAccels;
        public Vector3[] convertedAccels;
        public Vector3[] convertedAccelsAtCalibration;
        public Quaternion[] estimatedRotations;
        public Vector3[] rawGyros;
        public Vector3[] convertedGyros;
        
        public Kalman kalmanX;
        public Kalman kalmanY;
        public Kalman kalmanZ;

        private ulong previousTime;

        public Vector3 frankenAccelAtCalibration;
        public Vector3 frankenGravity;
        public Vector3 frankenAccel;
        public Vector3 frankenGyro;
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

            rawAccels = new Vector3[3];
            convertedAccels = new Vector3[3];
            convertedAccelsAtCalibration = new Vector3[3];
            estimatedRotations = new Quaternion[3];

            rawGyros = new Vector3[3];
            convertedGyros = new Vector3[3];

            frankenAccelAtCalibration = Vector3.zero;
            frankenGravity = Vector3.zero;
            frankenAccel = Vector3.zero;
            frankenGyro = Vector3.zero;
            frankenTranslationalAccel = Vector3.zero;
            frankenEstRotation = Quaternion.identity;
            frankenFilteredRotation = Quaternion.identity;
        }

        public void RequestMasterCalibration()
        {
            serial.DiscardInBuffer();
            serial.DiscardOutBuffer();
            serial.Write(calibrationMsg);
        }
        public void RequestSlave1Calibration()
        {
            serial.DiscardInBuffer();
            serial.DiscardOutBuffer();
            serial.Write(slave1CalibrationMsg);
        }
        public void RequestSlave2Calibration()
        {
            serial.DiscardInBuffer();
            serial.DiscardOutBuffer();
            serial.Write(slave2CalibrationMsg);
        }
        public void Begin()
        {
            serial.DiscardInBuffer();
            serial.DiscardOutBuffer();
            serial.Write(beginMsg);
        }
        public void Disconnect()
        {
            serial.DiscardInBuffer();
            serial.DiscardOutBuffer();
            serial.Write(disconnectionMsg);
        }

        public void UpdateData(bool forceRead = false)
        {
            if(messageHasBeenProcessed || forceRead)
            {
                readBytesFromSerial();
                opcode = getOpcode();
                timestamp = getTimestamp();
                rawAccels = getAccels();
                rawGyros = getGyros();
            }
        }

        public void UpdateDataAndProcess(bool forceRead = false)
        {
            if(messageHasBeenProcessed || forceRead)
            {
                readBytesFromSerial();
                opcode = getOpcode();
                timestamp = getTimestamp();
                rawAccels = getAccels();
                rawGyros = getGyros();
            }
            if (opcode == calibration)
            {
                CurieAccelerometerRange = getAccelRange();
                CurieGyroscopeRange = getGyroRange();
            }

            convertedAccels[0].Set(
                convertRawAcceleration(rawAccels[0].x),
                convertRawAcceleration(rawAccels[0].y),
                convertRawAcceleration(rawAccels[0].z)
                );
            convertedAccels[1].Set(
                convertRawAcceleration(rawAccels[1].x),
                convertRawAcceleration(rawAccels[1].y),
                convertRawAcceleration(rawAccels[1].z)
                );
            convertedAccels[2].Set(
                convertRawAcceleration(rawAccels[2].x),
                convertRawAcceleration(rawAccels[2].y),
                convertRawAcceleration(rawAccels[2].z)
                );
            convertedGyros[0].Set(
                convertRawGyroRad(rawGyros[0].x),
                convertRawGyroRad(rawGyros[0].y),
                convertRawGyroRad(rawGyros[0].z)
                );
            convertedGyros[1].Set(
                convertRawGyroRad(rawGyros[1].x),
                convertRawGyroRad(rawGyros[1].y),
                convertRawGyroRad(rawGyros[1].z)
                );
            convertedGyros[2].Set(
                convertRawGyroRad(rawGyros[2].x),
                convertRawGyroRad(rawGyros[2].y),
                convertRawGyroRad(rawGyros[2].z)
                );

            dt = (timestamp - previousTime) / 1000000.0f;
            previousTime = timestamp;

            if(opcode == calibration)
            {
                convertedAccelsAtCalibration[0].Set(convertedAccels[0].x, convertedAccels[0].y, convertedAccels[0].z);
            }
            else if(opcode == slave1Calibration)
            {
                convertedAccelsAtCalibration[1].Set(convertedAccels[1].x, convertedAccels[1].y, convertedAccels[1].z);
            }
            else if(opcode == slave2Calibration)
            {
                convertedAccelsAtCalibration[2].Set(convertedAccels[2].x, convertedAccels[2].y, convertedAccels[2].z);
            }

            estimatedRotations[0] = Quaternion.LookRotation(convertedAccels[0]);
            estimatedRotations[1] = Quaternion.LookRotation(convertedAccels[1]);
            estimatedRotations[2] = Quaternion.LookRotation(convertedAccels[2]);

            frankenAccel = (convertedAccels[0] + convertedAccels[1] + convertedAccels[2]) / 3.0f;
            frankenGyro = (convertedGyros[0] + convertedGyros[1] + convertedGyros[2]) / 3.0f;

            if (opcode == calibration)
            {
                frankenAccelAtCalibration = frankenAccel;
            }

            frankenEstRotation = Quaternion.FromToRotation(frankenAccelAtCalibration, frankenAccel);

            Vector3 frankenEulerEstRotation = frankenEstRotation.eulerAngles;

            if (opcode == calibration)
            {
                kalmanX.setAngle(frankenEulerEstRotation.x);
                kalmanY.setAngle(frankenEulerEstRotation.y);
                kalmanZ.setAngle(frankenEulerEstRotation.z);
            }

            Vector3 frankenEulerFilteredRotation = Vector3.zero;
            frankenEulerFilteredRotation.x = kalmanX.getAngle(frankenEulerEstRotation.x, frankenGyro.x, dt);
            frankenEulerFilteredRotation.y = kalmanY.getAngle(frankenEulerEstRotation.y, frankenGyro.y, dt);
            frankenEulerFilteredRotation.z = kalmanZ.getAngle(frankenEulerEstRotation.z, frankenGyro.z, dt);

            frankenFilteredRotation = Quaternion.Euler(frankenEulerFilteredRotation);

            frankenGravity = frankenFilteredRotation * frankenAccelAtCalibration;

            frankenTranslationalAccel = frankenAccel - frankenGravity;

            messageHasBeenProcessed = true;
            serial.DiscardInBuffer();
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

        public Vector3[] getAccels()
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

        public Vector3[] getGyros()
        {
            Vector3 G1 = Vector3.zero;
            Vector3 G2 = Vector3.zero;
            Vector3 G3 = Vector3.zero;
            G1.x = (float)BitConverter.ToInt16(message, 28);
            G1.y = (float)BitConverter.ToInt16(message, 30);
            G1.z = (float)BitConverter.ToInt16(message, 32);
            G2.x = (float)BitConverter.ToInt16(message, 34);
            G2.y = (float)BitConverter.ToInt16(message, 36);
            G2.z = (float)BitConverter.ToInt16(message, 38);
            G3.x = (float)BitConverter.ToInt16(message, 40);
            G3.y = (float)BitConverter.ToInt16(message, 42);
            G3.z = (float)BitConverter.ToInt16(message, 44);
            Vector3[] result = { G1, G2, G3 };
            return result;
        }

        public short getAccelRange()
        {
            return BitConverter.ToInt16(message, 2);
        }

        public short getGyroRange()
        {
            return BitConverter.ToInt16(message, 4);
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
            g *= Mathf.PI;
            g /= 180.0f;
            return g;
        }

    }

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
        curieData.UpdateDataAndProcess();
    }

    public bool CalibrateMaster()
    {
        int loops = 0;
        int tries = 0;

        while ((curieData.opcode != curieData.calibration) && (tries < 20))
        {
            if (debug)
            {
                Debug.Log("try: " + tries);
            }
            curieData.RequestMasterCalibration();
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
            curieData.UpdateDataAndProcess();
            return true;
        }
    }

    public bool CalibrateSlave1()
    {
        int loops = 0;
        int tries = 0;

        while ((curieData.opcode != curieData.slave1Calibration) && (tries < 20))
        {
            if (debug)
            {
                Debug.Log("try: " + tries);
            }
            curieData.RequestSlave1Calibration();
            while ((curieData.opcode != curieData.slave1Calibration) && (loops < 100))
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
        if (curieData.opcode != curieData.slave1Calibration)
        {
            return false;
        }
        else
        {
            if (debug)
            {
                Debug.Log("Calibration: " + curieData.getString());
            }
            curieData.UpdateDataAndProcess();
            return true;
        }
    }

    public bool CalibrateSlave2()
    {
        int loops = 0;
        int tries = 0;

        while ((curieData.opcode != curieData.slave2Calibration) && (tries < 20))
        {
            if (debug)
            {
                Debug.Log("try: " + tries);
            }
            curieData.RequestSlave2Calibration();
            while ((curieData.opcode != curieData.slave2Calibration) && (loops < 100))
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
        if (curieData.opcode != curieData.slave2Calibration)
        {
            return false;
        }
        else
        {
            if (debug)
            {
                Debug.Log("Calibration: " + curieData.getString());
            }
            curieData.UpdateDataAndProcess();
            return true;
        }
    }

    public void Disconnect()
    {
        curieData.Disconnect();
    }



}
