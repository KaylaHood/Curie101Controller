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
    private int serialMessageSize = 34;

    public CurieDataManager curieData;

    public bool isZeroMotion;

    public class CurieDataManager
    {
        public SerialPort serial;
        private int messageLength;
        private byte[] message;

        public int normal = 0;
        public int zeromotion = 1;
        public int calibration = 2;
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
        public Vector3[] rawAccels;
        public Vector3[] convertedAccels;
        public Vector3[] rotatedGravities;
        public Vector3[] translationalAccels;
        public Vector3[] convertedAccelsAtCalibration;
        public Quaternion[] estRotations;
        public Quaternion[] filteredRotations;
        public Quaternion[] estRotationsAtCalibration;
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

        public Vector3 frankenOriginalGravity;
        public Vector3 frankenGravity;
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

            rawAccels = new Vector3[3];
            convertedAccels = new Vector3[3];
            rotatedGravities = new Vector3[3];
            translationalAccels = new Vector3[3];
            convertedAccelsAtCalibration = new Vector3[3];
            estRotations = new Quaternion[3];
            filteredRotations = new Quaternion[3];
            estRotationsAtCalibration = new Quaternion[3];

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
            serial.DiscardOutBuffer();
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
                rawAccels = getAcc();
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
                rawAccels = getAcc();
                rawGyro = getGyro();
            }
            if (opcode == calibration)
            {
                CurieAccelerometerRange = rawGyro.x;
                CurieGyroscopeRange = rawGyro.y;
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
            convertedGyro.Set(
                convertRawGyroRad(rawGyro.x),
                convertRawGyroRad(rawGyro.y),
                convertRawGyroRad(rawGyro.z)
                );

            estRotations = new Quaternion[] {
                Quaternion.LookRotation(convertedAccels[0]),
                Quaternion.LookRotation(convertedAccels[1]),
                Quaternion.LookRotation(convertedAccels[2])
                };

            if (opcode == calibration)
            {
                estRotationsAtCalibration = estRotations;
                convertedAccelsAtCalibration = convertedAccels;
            }
            slave1RotationFromMaster = estRotationsAtCalibration[1] * (Quaternion.FromToRotation(convertedAccelsAtCalibration[0],convertedAccels[0]));
            slave2RotationFromMaster = estRotationsAtCalibration[2] * (Quaternion.FromToRotation(convertedAccelsAtCalibration[0],convertedAccels[0]));

            slave1RotatedAccel = slave1RotationFromMaster * convertedAccels[1];
            slave2RotatedAccel = slave2RotationFromMaster * convertedAccels[2];
            estRotations[1] = Quaternion.LookRotation(slave1RotatedAccel);
            estRotations[2] = Quaternion.LookRotation(slave2RotatedAccel);

            frankenAccel = (convertedAccels[0] + slave1RotatedAccel + slave2RotatedAccel) / 3;

            if(opcode == calibration)
            {
                frankenOriginalGravity = frankenAccel;
            }

            frankenEstRotation = Quaternion.LookRotation(frankenAccel);

            dt = (timestamp - previousTime) / 1000000.0f;
            previousTime = timestamp;

            Vector3 eulerFrankenRotation = frankenEstRotation.eulerAngles;
            kalmanX.setAngle(eulerFrankenRotation.x);
            kalmanY.setAngle(eulerFrankenRotation.y);
            kalmanZ.setAngle(eulerFrankenRotation.z);

            Vector3 frankenKalmanRot = Vector3.zero;
            frankenKalmanRot.x = kalmanX.getAngle(eulerFrankenRotation.x, convertedGyro.x, dt);
            frankenKalmanRot.y = kalmanY.getAngle(eulerFrankenRotation.y, convertedGyro.y, dt);
            frankenKalmanRot.z = kalmanZ.getAngle(eulerFrankenRotation.z, convertedGyro.z, dt);

            frankenFilteredRotation = Quaternion.Euler(frankenKalmanRot);

            Vector3 eulerMasterRotation = estRotations[0].eulerAngles;
            kalmanX.setAngle(eulerMasterRotation.x);
            kalmanY.setAngle(eulerMasterRotation.y);
            kalmanZ.setAngle(eulerMasterRotation.z);

            Vector3 masterKalmanRot = Vector3.zero;
            masterKalmanRot.x = kalmanX.getAngle(eulerMasterRotation.x, convertedGyro.x, dt);
            masterKalmanRot.y = kalmanY.getAngle(eulerMasterRotation.y, convertedGyro.y, dt);
            masterKalmanRot.z = kalmanZ.getAngle(eulerMasterRotation.z, convertedGyro.z, dt);

            filteredRotations[0] = Quaternion.Euler(masterKalmanRot);

            Vector3 eulerSlave1Rotation = estRotations[1].eulerAngles;
            kalmanX.setAngle(eulerSlave1Rotation.x);
            kalmanY.setAngle(eulerSlave1Rotation.y);
            kalmanZ.setAngle(eulerSlave1Rotation.z);

            Vector3 slave1KalmanRot = Vector3.zero;
            slave1KalmanRot.x = kalmanX.getAngle(eulerSlave1Rotation.x, convertedGyro.x, dt);
            slave1KalmanRot.y = kalmanY.getAngle(eulerSlave1Rotation.y, convertedGyro.y, dt);
            slave1KalmanRot.z = kalmanZ.getAngle(eulerSlave1Rotation.z, convertedGyro.z, dt);

            filteredRotations[1] = Quaternion.Euler(slave1KalmanRot);

            Vector3 eulerSlave2Rotation = estRotations[2].eulerAngles;
            kalmanX.setAngle(eulerSlave2Rotation.x);
            kalmanY.setAngle(eulerSlave2Rotation.y);
            kalmanZ.setAngle(eulerSlave2Rotation.z);

            Vector3 slave2KalmanRot = Vector3.zero;
            slave2KalmanRot.x = kalmanX.getAngle(eulerSlave2Rotation.x, convertedGyro.x, dt);
            slave2KalmanRot.y = kalmanY.getAngle(eulerSlave2Rotation.y, convertedGyro.y, dt);
            slave2KalmanRot.z = kalmanZ.getAngle(eulerSlave2Rotation.z, convertedGyro.z, dt);

            filteredRotations[2] = Quaternion.Euler(slave2KalmanRot);

            frankenFilteredRotation = Quaternion.Euler(frankenKalmanRot);
            frankenGravity = frankenFilteredRotation * frankenOriginalGravity;
            frankenTranslationalAccel = frankenAccel - frankenGravity;

            rotatedGravities[0] = filteredRotations[0] * originalGravityArray[0];
            rotatedGravities[1] = filteredRotations[1] * originalGravityArray[1];
            rotatedGravities[2] = filteredRotations[2] * originalGravityArray[2];

            translationalAccels[0] = convertedAccels[0] - rotatedGravities[0];
            translationalAccels[1] = slave1RotatedAccel - rotatedGravities[1];
            translationalAccels[2] = slave2RotatedAccel - rotatedGravities[2];

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
