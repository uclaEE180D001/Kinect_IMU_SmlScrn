using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BluetoothController;
using BluetoothController.Kinect;
using Microsoft.Kinect;
using System.Windows.Media.Media3D;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace BluetoothController
{
    public static class MathFunctions
    {
        public enum Plane
        { xy, xz, yz}

        /// <summary>
        /// Compute the angle between 3D vectors on a 2D plane
        /// </summary>
        /// <param name="aVec1"></param>
        /// <param name="aVec2"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        public static double angleBetweenVectors(double[] aVec1, double[] aVec2, Plane p)
        {
            double sum = 0, m1 = 0, m2 = 0, angle = 0;
            int a = 0, b = 0;

            switch (p)
            {
                case Plane.xy:
                    { 
                        a = 0; 
                        b = 1; 
                        break;
                    }
                case Plane.xz:
                    {
                        a = 0;
                        b = 2;
                        break;
                    }
                case Plane.yz:
                    {
                        a = 1;
                        b = 2;
                        break;
                    }
            }


            //dot product
            sum += aVec1[a] * aVec2[a];
            sum += aVec1[b] * aVec2[b];

            //magnitude
            m1 = Math.Sqrt((aVec1[a] * aVec1[a]) + (aVec1[b] * aVec1[b]));
            m2 = Math.Sqrt((aVec2[a] * aVec2[a]) + (aVec2[b] * aVec2[b]));

            angle = Math.Acos(sum / (m1 * m2));
            return angle; //radians? 
        }

        public static double[] angleBetweenVectors(double[] a, double[] b)
        {
            double[] anorm = normalizeVector(a);
            double[] bnorm = normalizeVector(b);

            double[] axb = crossProduct(anorm, bnorm);
            double[] axbnorm = normalizeVector(axb);

            double adotb = dotProduct(anorm, bnorm);

            double angle = Math.Acos(adotb);

            double s = Math.Sin(angle);
            double c = Math.Cos(angle);
            double t = 1 - c;

            double x = axbnorm[0];
            double y = axbnorm[1];
            double z = axbnorm[2];

            double[,] R = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

            R[0, 0] = t * x * x + c;
            R[1, 0] = t * x * y + s * z;
            R[2, 0] = t*x*z - s*y;
            
            R[0, 1] = t*x*y - s*z;
            R[1, 1] = t*y*y + c;
            R[2, 1] = t * y * z + s * x;
            
            R[0, 2] = t*x*z +s*y;
            R[1, 2] = t*y*z -s*x;
            R[2, 2] = t * z * z + c;

            double[] output = new double[3]; //rotation around x,y,z
            output[0] = Math.Atan2( R[2,1] , R[2,2]) *180/Math.PI;
            output[1] = Math.Atan2(-R[2, 0], Math.Sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2])) * 180 / Math.PI;
            output[2] = Math.Atan2(R[1, 0], R[0, 0]) * 180 / Math.PI;

            return output;
        }

        public static double[] rotationBetweenQuaternions(Quaternion a, Quaternion b)
        {
            Quaternion rq;

            b.Invert();

            rq = Quaternion.Multiply(b, a);

            double[] result = new double[4];
            result[0]=rq.W;
            result[1] = rq.X;
            result[2] = rq.Y;
            result[3] = rq.Z;

            return  result;
        }

        public static double[] angleBetweenVectorsQuat(double[] a, double[] b)
        {
            double[] anorm = normalizeVector(a);
            double[] bnorm = normalizeVector(b);

            double[] axb = crossProduct(anorm, bnorm);
            double[] axbnorm = normalizeVector(axb);

            double adotb = dotProduct(anorm, bnorm);

            double angle = Math.Acos(adotb);

            double x = axbnorm[0];
            double y = axbnorm[1];
            double z = axbnorm[2];



            double[] output = new double[4]; //rotation quaternion
            output[0] = Math.Cos(angle/2);
            output[1] = -x*Math.Sin(angle/2);
            output[2] = -y*Math.Sin(angle/2);
            output[3] = -z * Math.Sin(angle / 2);


            return output;
        }

        public enum MatrixType
        { Roll, Yaw }

        /// <summary>
        /// Compute an average matrix for all matrices in the calibrators buffer
        /// </summary>
        /// <param name="calibrators"></param>
        /// <param name="mt"></param>
        /// <returns></returns>
        public static double[,] matrixAvg(CircularBuffer<SensorCalibratorData> calibrators, MatrixType mt)
        {
            double[,] aggregateMatrix = new double[,] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
            double[,] avgMatrix = new double[,] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
            double[,] UsableSize = new double[,] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
            for (int i = 0; i < calibrators.Capacity; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                    {
                        if (mt == MatrixType.Roll && !double.IsNaN(calibrators[i].rollCorrection[j,k]))
                        {
                            UsableSize[j, k]++;
                            aggregateMatrix[j, k] += calibrators[i].rollCorrection[j, k];
                        }
                        else if (mt == MatrixType.Yaw && !double.IsNaN(calibrators[i].yawCorrection[j, k]))
                        {
                            UsableSize[j, k]++;
                            aggregateMatrix[j, k] += calibrators[i].yawCorrection[j, k];
                        }
                    }

            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    avgMatrix[j, k] = aggregateMatrix[j, k] / UsableSize[j, k];

            return avgMatrix;
        }

        /// <summary>
        /// Compute an average linear correction constant for all constants in the calibrators buffer
        /// </summary>
        /// <param name="calibrators"></param>
        /// <returns></returns>
        public static double correctionConstAvg(CircularBuffer<SensorCalibratorData> calibrators)
        {
            double aggregateConst = 0;
            double UsableSize = 0;
            for (int i = 0; i < calibrators.Capacity; i++)
            {
                if (calibrators[i].correctionConst != double.NaN)
                {
                    aggregateConst += calibrators[i].correctionConst;
                    UsableSize++;
                }
            }

            return aggregateConst / UsableSize;
        }


        public static double linearDisplacementAvg(CircularBuffer<SensorCalibratorData> calibrators)
        {
            double aggregateDisplacement = 0;
            double UsableSize = 0;
            for (int i = 0; i < calibrators.Capacity; i++)
            {
                if (calibrators[i].correctionConst != double.NaN)
                {
                    aggregateDisplacement += calibrators[i].Displacement;
                    UsableSize++;
                }
            }

            return aggregateDisplacement / UsableSize;
        }


        public static double[] eulerAngleAvg(CircularBuffer<SensorCalibratorData> calibrators)
        {
            double[] sums = new double[3];
            double[] output = new double[3];
            for (int i = 0; i < calibrators.Capacity; i++)
            {
                for(int j = 0; j<3;j++){
                    sums[j] += calibrators[i].eulerAngles[j];
                }
            }

            for (int j = 0; j < 3; j++)
            {
                output[j] = sums[j] / calibrators.Capacity;
            }

            return output;
        }

        public static double[] rotQuatAvg(CircularBuffer<SensorCalibratorData> calibrators)
        {
            
            double[] sums = new double[4];
            double[] output = new double[4];
            for (int i = 0; i < calibrators.Capacity; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    sums[j] += calibrators[i].rotquat[j] * calibrators[i].rotquat[j];
                }
            }

            for (int j = 0; j < 4; j++)
            {
                output[j] = Math.Sqrt(sums[j] / calibrators.Capacity);
            }
            
             return output;
         

            //adapted from Tolga Birdal
            /*
            Based on 
            Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman. 
            "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30, 
            no. 4 (2007): 1193-1197.
             * */
            /*
            int M = calibrators.size;
            //A = zeros(4,4)
            double[,] A = new double[,] { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

            for (int i = 0; i < M; i++)
            {
                //q = Q(i,:)';
                double[] q = new double[4];
                for (int j = 0; j < 4; j++ )
                    q[j] = calibrators[i].rotquat[j];

                //A = q*q'+A; % rank 1 update
                for (int j = 0; j < 4; j++)
                    for (int k = 0; k < 4; k++)
                    {
                        //q*q'[j, k] = q[j] * q[k];
                        A[j, k] += q[j] * q[k];
                    }
            }

            //% scale
            //A=(1.0/M)*A;
            double s = 1.0/M;
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    A[j, k] *= s;

            //% Get the eigenvector corresponding to largest eigen value
            //[Qavg, Eval] = eigs(A,1);
            double[] wr, wi;
            double[,] vl, vr;
            alglib.rmatrixevd(A, 4, 3, out wr,out wi,out vl,out vr);

            ShowArrayInfo(wr);
            ShowArrayInfo(wi);
            
            ShowArrayInfo(vl);
            ShowArrayInfo(vr);

            double[] Qavg = new double[4];
            for (int i = 0; i < 4; i++)
                Qavg[i] = wr[i];

            return Qavg;
             * */
        }

        private static void ShowArrayInfo(Array arr)
        {
            Console.WriteLine("Length of Array:      {0,3}", arr.Length);
            Console.WriteLine("Number of Dimensions: {0,3}", arr.Rank);
            // For multidimensional arrays, show number of elements in each dimension. 
            if (arr.Rank > 1)
            {
                for (int dimension = 1; dimension <= arr.Rank; dimension++)
                    Console.WriteLine("   Dimension {0}: {1,3}", dimension,
                                      arr.GetUpperBound(dimension - 1) + 1);
            }
            Console.WriteLine();
        }


    /*    public static double[] quatByRotAvg(CircularBuffer<SensorCalibratorData> calibrators)
        {
            double[] sums = new double[4];
            double[] output = new double[4];
            for (int i = 0; i < calibrators.Capacity; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    sums[j] += calibrators[i].quatByRotations[j];
                }
            }

            for (int j = 0; j < 4; j++)
            {
                output[j] = sums[j] / calibrators.Capacity;
            }

            return output;
        }*/
        


        /// <summary>
        /// This can be used to get the location of the point we are interested in between two joints
        /// </summary>
        /// <param name="j1">Usually further (FROM THE BODY) joint</param>
        /// <param name="j2">usually closer joint</param>
        /// <param name="pos">0 means at j1, 100 means at j2</param>
        /// <returns>a 3 vector with the location of the point of interest</returns>
        public static double[] midpoint(Joint j1, Joint j2, int pos)
        {
            double[] midPos = new double[3];

            //X
            midPos[0] = (j2.Position.X - j1.Position.X) * (double)(pos / 100) + j1.Position.X;
            //Y
            midPos[1] = (j2.Position.Y - j1.Position.Y) * (double)(pos / 100) + j1.Position.Y;
            //Z
            midPos[2] = (j2.Position.Z - j1.Position.Z) * (double)(pos / 100) + j1.Position.Z;

            return midPos;
        }


        /// <summary>
        /// Performs a rotation that maintains vector magnitude
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static Quaternion HamiltonianMultiply(Quaternion a, Quaternion b)
        {
            Quaternion temp = a * b;
            a.Invert();
            return temp * a;
        }

        public static double[,] QuaternionToMatrix(Quaternion q)
        {
            double[,] matrix = new double[3, 3];
            double qx = q.X, qy = q.Y, qz = q.Z, qw = q.W;
            matrix[0, 0] = 1 - 2 * qy * qy - 2 * qz * qz;
            matrix[0, 1] = 2 * qx * qy - 2 * qz * qw;
            matrix[0, 2] = 2 * qx * qz + 2 * qy * qw;
            matrix[1, 0] = 2 * qx * qy + 2 * qz * qw;
            matrix[1, 1] = 1 - 2 * qx * qx - 2 * qz * qz;
            matrix[1, 2] = 2 * qy * qz - 2 * qx * qw;
            matrix[2, 0] = 2 * qx * qz - 2 * qy * qw;
            matrix[2, 1] = 2 * qy * qz + 2 * qx * qw;
            matrix[2, 2] = 1 - 2 * qx * qx - 2 * qy * qy;

            return matrix;
        }

        public static double[] MatrixVectorMultiply(double[] vector, double[,] matrix3Dim)
        {
            double[] output = new double[] { 0, 0, 0 };
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    output[i] += matrix3Dim[i, j] * vector[j];
            return output;
        }

        //Normalize a 3D Vector
        public static double[] normalizeVector(double[] v)
        {
            double[] norm = new double[] { 0, 0, 0 };
            Vector3D normv = new Vector3D(v[0], v[1], v[2]);
            normv.Normalize();

            norm[0] = normv.X;
            norm[1] = normv.Y;
            norm[2] = normv.Z;

            return norm;
        }

        //Cross product of two 3d vectors
        public static double[] crossProduct(double[] a, double[] b)
        {
            Vector3D av = new Vector3D(a[0],a[1],a[2]);
            Vector3D bv = new Vector3D(b[0],b[1],b[2]);

            Vector3D ov = Vector3D.CrossProduct(av,bv);//output as Vector3D
            double[] output = new double[] { 0, 0, 0 };//output as double[]
            output[0] = ov.X;
            output[1] = ov.Y;
            output[2] = ov.Z;

            return output;
        }

        //dot product of two 3d vectors
        public static double dotProduct(double[] a, double[] b)
        {
            Vector3D av = new Vector3D(a[0], a[1], a[2]);
            Vector3D bv = new Vector3D(b[0], b[1], b[2]);

            return Vector3D.DotProduct(av,bv);
        }

        


    }

    //[global::Microsoft.VisualStudio.TestTools.UnitTesting.TestClass]
    //public class MyTestClass
    //{

    // [TestMethod]
    //  public void MyTestMethod()
    //  {
    //      double[,] testMatrix = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    //      double[,] testMatrix2 = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    //      Assert.AreEqual(testMatrix, testMatrix2);

        
    //  }   
    //}
}

