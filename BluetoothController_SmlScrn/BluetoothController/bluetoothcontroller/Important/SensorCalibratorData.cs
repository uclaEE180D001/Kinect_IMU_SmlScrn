using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BluetoothController;
using BluetoothController.Kinect;
using System.Windows.Media.Media3D;

namespace BluetoothController 
{
 
    /// <summary>
    /// Calculates correction variables to virtually adjust motion sensors into the desired orientation and position
    /// </summary>
    public class SensorCalibratorData : IData
    {
        private double[] actualLinearAcc;
        private double[] idealLinearAcc;
        private double[] correctedLinearAcc; //should be ideal
        private double[] angularAcc;
        private double linearCorrectionConst;
        private double linearDisplacement;
        private double[,] yawCorrectionMatrix;
        private double[,] rollCorrectionMatrix;
        private double yawAngle;
        private double rollAngle;
        public double[] eulerAngles;
        public double[] rotquat;

        public bool CalibrationFailed { get; set; }
        public bool DataIsGood { get; set; }

      
        public SensorCalibratorData()
        {
            linearCorrectionConst = 1;
            linearDisplacement = 0;
            yawCorrectionMatrix = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            rollCorrectionMatrix = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            actualLinearAcc = new double[] { 0, 0, 0 };
            idealLinearAcc = new double[] { 0, 0, 0 };
            correctedLinearAcc = new double[] { 0, 0, 0 };
            angularAcc = new double[] { 0, 0, 0 };
            yawAngle = 0;
            rollAngle = 0;
            CalibrationFailed = false;
        }

        public void setData(double[] actualVec, double[] angularActualVec, double[] idealVec, bool goodData )
        {
            angularAcc = angularActualVec;
            actualLinearAcc = actualVec;
            idealLinearAcc = idealVec;
            DataIsGood = goodData;
        }

        /// <summary>
        /// Computes a linear correction constant that vertically shifts the IMUs 
        /// </summary>
        public void generateLinearCorrection()
        {
            //[0] = x, [1] = y, [2] = z
            double actualAvgRadius = 0;
            double idealAvgRadius = 0;
            double actualAvgRadiusAggregate = 0, linearAvgRadiusAggregate = 0;

            //Calculation of correctionConst
            for (int i = 0; i < 3; i++)
            {
                actualAvgRadiusAggregate += actualLinearAcc[i] / angularAcc[i];
                linearAvgRadiusAggregate += idealLinearAcc[i] / angularAcc[i];
            }

            actualAvgRadius = actualAvgRadiusAggregate / 3;
            idealAvgRadius = linearAvgRadiusAggregate / 3;

            linearDisplacement = Math.Abs(idealAvgRadius - actualAvgRadius);
            linearCorrectionConst = idealAvgRadius / actualAvgRadius;
        }

        /// <summary>
        ///Generates a rotation matrix that rotates acceleration vectors in the xy plane (yaw)
        /// </summary>
        public void generateYawCorrection()
        {
            yawAngle = MathFunctions.angleBetweenVectors(actualLinearAcc, idealLinearAcc, MathFunctions.Plane.xy);
            double cosRot = Math.Cos(yawAngle);
            double sinRot = Math.Sin(yawAngle);

            yawCorrectionMatrix[0, 0] = cosRot;
            yawCorrectionMatrix[0, 1] = -sinRot;
            yawCorrectionMatrix[0, 2] = 0;
            yawCorrectionMatrix[1, 0] = sinRot;
            yawCorrectionMatrix[1, 1] = cosRot;
            yawCorrectionMatrix[1, 2] = 0;
            yawCorrectionMatrix[2, 0] = 0;
            yawCorrectionMatrix[2, 1] = 0;
            yawCorrectionMatrix[2, 2] = 1;

        }


        /// <summary>
        ///Generates a rotation matrix that rotates acceleration vectors in the xz plane (roll)
        /// </summary>
        public void generateRollCorrection()
        {
            rollAngle = MathFunctions.angleBetweenVectors(actualLinearAcc, idealLinearAcc, MathFunctions.Plane.xz);
            double cosRot = Math.Cos(rollAngle);
            double sinRot = Math.Sin(rollAngle);


            rollCorrectionMatrix[0, 0] = cosRot;
            rollCorrectionMatrix[0, 1] = 0;
            rollCorrectionMatrix[0, 2] = sinRot;
            rollCorrectionMatrix[1, 0] = 0;
            rollCorrectionMatrix[1, 1] = 1;
            rollCorrectionMatrix[1, 2] = 0;
            rollCorrectionMatrix[2, 0] = -sinRot;
            rollCorrectionMatrix[2, 1] = 0;
            rollCorrectionMatrix[2, 2] = cosRot;
        }


        public double ExtractYawAngle()
        { 
            //return Math.Acos(yawCorrectionMatrix[1, 1]) * (180.0/Math.PI); 
            return eulerAngles[2];
        
        }

        public double ExtractRollAngle()
        { 
            //return Math.Acos(rollCorrectionMatrix[0, 0]) * (180.0 / Math.PI); 
            return eulerAngles[0];
        
        }

        public double ExtractPitchAngle()
        { return eulerAngles[1]; }


        public double correctionConst
        {
            get { return linearCorrectionConst;  }
            set { linearCorrectionConst = value; }
        }

        public double Displacement
        {
            get { return linearDisplacement; }
            set { linearDisplacement = value; }
        }

        public double[,] yawCorrection
        {
            get { return yawCorrectionMatrix; }
            set { yawCorrectionMatrix = value;  }
        }

        public double[,] rollCorrection
        {
            get { return rollCorrectionMatrix; }
            set { rollCorrectionMatrix = value; }
        }

        /// <summary>
        /// Generates corrections for linear, yaw, and roll misplacements
        /// </summary>
        public void generateCorrections()
        {
            generateLinearCorrection();
            //generateYawCorrection();
            //generateRollCorrection();
            eulerAngles = MathFunctions.angleBetweenVectors(actualLinearAcc, idealLinearAcc);
            rotquat = MathFunctions.angleBetweenVectorsQuat(actualLinearAcc, idealLinearAcc);
            //quatByRotations = MathFunctions.rotationBetweenQuaternions(actualQuat, idealQuat);
        }

        /// <summary>
        /// Applies the correction varibles to future raw data
        /// </summary>
        /// <param name="actualAcc"> This parameter represents a raw acceleration that is not the same acceleration as the actualLinearAcceleration member</param>
        /// <returns></returns>
        public double[] transform(double[] actualAcc)
        {
            double[] tempAcc = new double[] {0,0,0};
            double[] tempAcc2 = new double[] {0,0,0};

            //linear correction
            for (int i = 0; i < 3; i++)
                tempAcc[i] = actualAcc[i] * linearCorrectionConst;

            //yaw correction
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    tempAcc2[i] += yawCorrectionMatrix[i, j] * tempAcc[j];

            //roll correction
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    correctedLinearAcc[i] += rollCorrectionMatrix[i, j] * tempAcc2[j];

            return correctedLinearAcc;
        }


        public string ToWindowsFormString()
        {
            throw new NotImplementedException();
        }

        public string ToFileString()
        {
            throw new NotImplementedException();
        }


        public string ToPreviewString()
        {
            StringBuilder sb = new StringBuilder();
           // sb.AppendFormat(
            //    "Linear Displacement(m): {0}\n, Yaw (deg): {1}\n, Roll (deg): {2}, Pitch (deg): {3}, Q0: {4}, Q1: {5}. Q2: {6}. Q3: {7} QN0: {8}, QN1: {9}, QN2: {10}, QN3: {11},"
           //     , linearDisplacement, ExtractYawAngle(), ExtractRollAngle(), ExtractPitchAngle(), rotquat[0], rotquat[1], rotquat[2], rotquat[3], quatByRotations[0], quatByRotations[1], quatByRotations[2], quatByRotations[3]);
              sb.AppendFormat(
                "Linear Displacement(m): {0}\r\nYaw (deg): {1}\r\nRoll (deg): {2}\r\nPitch (deg): {3}\r\nQ0: {4}\r\nQ1: {5}\r\nQ2: {6}\r\nQ3: {7}"
                 , linearDisplacement, ExtractYawAngle(), ExtractRollAngle(), ExtractPitchAngle(), rotquat[0], rotquat[1], rotquat[2], rotquat[3]);
            return sb.ToString();
        }


        public long NowInTicks
        {
            get
            {
                throw new NotImplementedException();
            }
            set
            {
                throw new NotImplementedException();
            }
        }

        #region IData Members


        public string ToFileHeaderString()
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
