using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BluetoothController;
using BluetoothController.Kinect;
using Microsoft.Kinect;
using System.Windows.Media.Media3D;


namespace BluetoothController
{
    /// <summary>
    /// Joins virtual and actual acceleration data for calibration
    /// </summary>
    public class Merge
    {
        private IDataProducer<InertialSensorData> ActualSensor;
        private IDataProducer<VirtualSensorData> IdealSensor;
        public event EventHandler<SensorCalibratorData> CalibrationComplete;
        CircularBuffer<AccelerationTime> virtualAcc; //kinect 
        CircularBuffer<AccelerationTime> actualAcc; //sensor
        CircularBuffer<SensorCalibratorData> calibrators;
        SensorCalibratorData finalCalibrator;
        private const double KinectFPS = 30;
        private const double InertialMPS = 200;
        public const double CalibrationLookBackTimeInSec = 2;
        private bool LastVirtualFiltered = false;
        //private int CalibrationCount = 0;   //locally stored count on the number of times calibrator has been called. 


        public Merge(IDataProducer<VirtualSensorData> idealsensor, IDataProducer<InertialSensorData> actualsensor)
        {
            //CalibrationCount = count;
            this.ActualSensor = actualsensor;
            IdealSensor = idealsensor;

            ActualSensor.NewTData += OnActualSensorNewTData;
            IdealSensor.NewTData += OnIdealSensorNewTData;
            virtualAcc = new CircularBuffer<AccelerationTime>((int) (KinectFPS * CalibrationLookBackTimeInSec)); //kinect
            actualAcc = new CircularBuffer<AccelerationTime>((int) (InertialMPS * (CalibrationLookBackTimeInSec + 1))); //sensor
            calibrators = new CircularBuffer<SensorCalibratorData>((int) (KinectFPS * CalibrationLookBackTimeInSec - 4)); //calibrators
            finalCalibrator = new SensorCalibratorData();

            //DataTracker.CurrentSection = 999;
        }

        ~Merge()
        {
            ActualSensor.NewTData -= this.OnActualSensorNewTData;
            IdealSensor.NewTData -= this.OnIdealSensorNewTData;
        }

        public bool IsKinectBufferFull()
        {
            return virtualAcc.IsFull;
        }

        public int GetKinectBufferSize()
        {
            return virtualAcc.size;
        }

        public int GetKinectBufferCapacity()
        {
            return virtualAcc.Capacity;
        }

        public bool IsIMUBufferFull()
        {
            return actualAcc.IsFull;
        }

        public int GetIMUBufferSize()
        {
            return actualAcc.size;
        }

        public int GetIMUBufferCapacity()
        {
            return actualAcc.Capacity;
        }


        /// <summary>
        /// Stores virtual accleration data into buffer
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void OnIdealSensorNewTData(object sender, VirtualSensorData e)
        {
            //TO DO: IF data gets pulled into buffer, gets tagged with section ID? Wait.. thought we were tagging all data touched??
                //regardless, no longer doing this here. 
            //e.section = this.CalibrationCount;
            //DataTracker.CurrentSection = DataTracker.SectionCounter;
            //derivation and store into buffer
            AccelerationTime t = new AccelerationTime();

           

            if (ICherryPicker.isDataGood(e.acceleration, !e.isinferredornottracked))
            {
                //IF THIS CONDITIONAL IS CHANGED, IT MUST ALSO BE CHANGED IN THE DATA PRODUCERS. 
                t.setVal(e.acceleration, e.NowInTicks, !e.isinferredornottracked);  //if it's good, assign the proper current value - maybe we need to move this if statement to sensor producer
                virtualAcc.Enqueue(t);
                LastVirtualFiltered = false;
            }
            else
            {
                LastVirtualFiltered = true;
                //DataTracker.CurrentSection = 0;
            }
        }

        System.Threading.Thread WorkerThread;
        System.Threading.ThreadStart WorkerFunction;

        /// <summary>
        /// Build asychronous thread to perform calibration
        /// </summary>
        /// <returns></returns>
        public Merge Calibrate()
        {
            //this.CalibrationCount++;
            if(WorkerThread != null)
                if (WorkerThread.IsAlive)
                    this.WorkerThread.Abort();  
            WorkerFunction = new System.Threading.ThreadStart(() =>
            {
                this.mathCalibrate();
                if(this.CalibrationComplete != null)
                    this.CalibrationComplete(this, this.finalCalibrator);
                ActualSensor.NewTData += this.OnActualSensorNewTData;
                IdealSensor.NewTData += this.OnIdealSensorNewTData;
            });
            this.WorkerThread = new System.Threading.Thread(WorkerFunction);
            IdealSensor.NewTData -= this.OnIdealSensorNewTData;
            //System.Threading.Thread.Sleep(200);
            ActualSensor.NewTData -= this.OnActualSensorNewTData;

            this.WorkerThread.Start();
            return this;

        }

        /// <summary>
        /// Grabs data from accelerometer and gyroscope,normalizes it, and stores it into buffer for comparison later
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void OnActualSensorNewTData(object sender, InertialSensorData e)
        {
            //TO DO: If data gets pulled into buffer for comparison, it gets tagged with section ID

            //getting acceleration data from accelerometer/gyroscope
            const double angularConst = Math.PI / (16.4 * 180.0);

            double xAcc = e.AccelerationAsVMD3.X;
            double yAcc = e.AccelerationAsVMD3.Y;
            double zAcc = e.AccelerationAsVMD3.Z;


            double xRot = (double)e.gyropscopes[0] * angularConst;
            double yRot = (double)e.gyropscopes[1] * angularConst;
            double zRot = (double)e.gyropscopes[2] * angularConst;

            double[] tempAngularActualAcc = new double[] { xRot, yRot, zRot };
            double[] tempActualAcc = new double[] { xAcc, yAcc, zAcc };
            AccelerationTime tempActualAccTime = new AccelerationTime();
            tempActualAccTime.setVal(tempActualAcc, tempAngularActualAcc, e.NowInTicks);

            if (!LastVirtualFiltered)
            {
                actualAcc.Enqueue(tempActualAccTime);
            }
        }

        
       
        /// <summary>
        /// Look back at X amount of seconds of acceleration data and generate appropriate corrections matrices and constants
        /// </summary>
        public void mathCalibrate()
        {
            int actualAccIndex = 0; 
            double[] avgActualAcc = new double[] { 0, 0, 0 };
            double[] avgAngularAcc = new double[] { 0, 0, 0 };
            double[] idealAcc = new double[] { 0, 0, 0 };
            int a = 0;
            finalCalibrator = new SensorCalibratorData();
            CreateFile();
            //NEED TO CONVERT FROM VIRTUAL TO IDEAL 

            for (int i = 0; i < calibrators.Capacity && i + 2 < virtualAcc.size; i++)
            {
                int virtualAccIndex = i + 2;
                try
                {
                    actualAccIndex = searchClosestTime(virtualAcc[i].Time, ref a); //Searches for the closest time in the actualAcc buffer to begin the calibration
                }
                catch
                {
                    finalCalibrator.CalibrationFailed = true;
                    //return;
                }
                avgActualAcc = averageActualAcc(actualAccIndex); //average 6 sensor accelertion vectors
                avgAngularAcc = averageAngularAcc(actualAccIndex);
                calibrators[i] = new SensorCalibratorData();
                calibrators[i].setData(avgActualAcc, avgAngularAcc, virtualAcc[virtualAccIndex + 1].linearAcceleration,
                    virtualAcc[virtualAccIndex + 1].IsGood); // Virtual is set back a frame to compensate for lag
                calibrators[i].generateCorrections();
                Write(avgActualAcc, virtualAcc[virtualAccIndex + 1].linearAcceleration);
            }

            Close();
            finalCalibrator.correctionConst = MathFunctions.correctionConstAvg(calibrators);
            finalCalibrator.Displacement = MathFunctions.linearDisplacementAvg(calibrators);
            //finalCalibrator.rollCorrection = MathFunctions.matrixAvg(calibrators, MathFunctions.MatrixType.Roll);
            //finalCalibrator.yawCorrection = MathFunctions.matrixAvg(calibrators, MathFunctions.MatrixType.Yaw);
            finalCalibrator.eulerAngles = MathFunctions.eulerAngleAvg(calibrators);
            finalCalibrator.rotquat = MathFunctions.rotQuatAvg(calibrators);
        }
        private StreamWriter write;
        void CreateFile()
        {
            //DEBUG: This is commented out as it was used for testing.
            //string filelocation = @"C:\Users\User\Documents\WHI\Act_vs_Virt" + DateTime.Now.Ticks.ToString() + ".csv";
            //    this.write = new StreamWriter(filelocation);
            //this.write.Write(String.Format("{0}, {1}, {2}, {3}, {4}, {5}",
            //    "Actual_ax",
            //    "Actual_ay",
            //    "Actual_az",
            //    "Virtual_ax",
            //    "Virtual_ay",
            //    "Virtual_az"));
            //this.write.Write(Environment.NewLine);
        }
        void Write(double[] actuala, double[] virtuala)
        {
            //DEBUG: This is commented out as it was used for testing.
            //this.write.Write(String.Format("{0}, {1}, {2}, {3}, {4}, {5}",
            //    actuala[0],
            //    actuala[1],
            //    actuala[2],
            //    virtuala[0],
            //    virtuala[1],
            //    virtuala[2]));
            //this.write.Write(Environment.NewLine);
        }
        void Close()
        {
            //DEBUG: This is commented out as it was used for testing.
            //try
            //{
            //    this.write.Flush();
            //    this.write.Close();
            //}
            //catch(Exception)
            //{}
        }

        /// <summary>
        /// Searches for closest matching times between the actual and virtual acceleration buffers 
        /// </summary>
        /// <param name="targetTime"></param>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <returns></returns>
        public int searchClosestTime(long targetTime, ref int startIndex)
        {
            int i = startIndex;
            while (actualAcc[i].Time < targetTime && i < actualAcc.Capacity)
                i++;

            if (i == virtualAcc.Capacity || i == 0 || i == 1)
                throw new Exception("Could not find matching time between virtual and acceleration vectors.");

            startIndex = i;

            return i;
        }


        /// <summary>
        /// Averages the six closest IMU accelerations to the corresponding virtual acceleration
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public double[] averageActualAcc(int index)
        {
            double totalX = 0, totalY = 0, totalZ = 0;
            for (int i = 0; i < 6; i++)
            {
                totalX += actualAcc[index + (i - 2)].linearAcceleration[0];
                totalY += actualAcc[index + (i - 2)].linearAcceleration[1];
                totalZ += actualAcc[index + (i - 2)].linearAcceleration[2];
            }

            return new double[] { totalX / 6, totalY / 6, totalZ / 6 };
        }

        public double[] averageAngularAcc(int index)
        {
            double totalX = 0, totalY = 0, totalZ = 0;
            for (int i = 0; i < 6; i++)
            {
                totalX += actualAcc[index + (i - 2)].angularAcceleration[0];
                totalY += actualAcc[index + (i - 2)].angularAcceleration[1];
                totalZ += actualAcc[index + (i - 2)].angularAcceleration[2];
            }

            return new double[] { totalX / 6, totalY / 6, totalZ / 6 };
        }
    }
}
