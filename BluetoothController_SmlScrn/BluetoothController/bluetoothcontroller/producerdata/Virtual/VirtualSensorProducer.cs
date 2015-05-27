using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BluetoothController;
using BluetoothController.Kinect;
using Microsoft.Kinect;
//using Altaxo.Calc.Regression;

namespace BluetoothController
{
    public class VirtualSensorProducer : IDataProducer<VirtualSensorData>, IDataProducer
    {



        public static VirtualSensorProducer GetVirtualSensorHelper(IDataProducer<KinectData> kinectposdata, SensorLocation sl)
        {
            switch(sl)
            {
                case SensorLocation.NotApplicable:
                    return null;
                case SensorLocation.NotSet:
                    return null;
                case SensorLocation.ForearmRight:
                    return new VirtualSensorProducer(kinectposdata, JointType.ElbowRight, JointType.WristRight, 0);
                case SensorLocation.ForearmLeft:
                    return new VirtualSensorProducer(kinectposdata, JointType.WristLeft, JointType.ElbowLeft, 0);
                case SensorLocation.UpperArmRight:
                    return new VirtualSensorProducer(kinectposdata, JointType.ElbowRight, JointType.ShoulderRight, 0);
                case SensorLocation.UpperArmLeft:
                    return new VirtualSensorProducer(kinectposdata, JointType.ElbowLeft, JointType.ShoulderLeft, 0);
                case SensorLocation.AnkleRight:
                    return new VirtualSensorProducer(kinectposdata, JointType.KneeRight, JointType.AnkleRight, 0);
                case SensorLocation.AnkelLeft:
                    return new VirtualSensorProducer(kinectposdata, JointType.KneeLeft, JointType.AnkleLeft, 0);
                default:
                    throw new InvalidOperationException("Bad Enum given as param");
            }
        }
        private IDataProducer<KinectData> KinectPosData;
        private IFilter X_Filter;
        private IFilter Y_Filter;
        private IFilter Z_Filter;
        private double[] pos;
        public event EventHandler<VirtualSensorData> NewTData;
        public event EventHandler<IData> NewIData;
        public SensorType SensorType { get { return SensorType.Virtual; } }
        public bool IsGood { get;  protected set; }
        public string DeviceName { get { return "KinectVirtualSensor"; } }
        public string DeviceAddress { get { return "KinectVirtualSensor"; } }
        private JointType CloserJoint;
        private JointType FurtherJoint;
        private int PercentDistance;
        private readonly double[] alpha = new double[]{ 0.4, 0.8, 0.4 }; // [0] = X, [1] = Y, [2] = Z
        private readonly double[] gamma = new double[] { 0.4, 0.0, 0.0 }; 
        private const NDerivative nd = NDerivative.second;

        public enum FilterType
        {
            SGF,
            OurSGF,
            DoubleExponential
        }
       
        public VirtualSensorProducer(IDataProducer<KinectData> kinectposdata, JointType closerjoint, JointType furtherjoint, int percentdistnace, FilterType ft = FilterType.DoubleExponential)
        {
            KinectPosData = kinectposdata;
            pos = new double[] { 0, 0, 0 };
            IsGood = true;
            KinectPosData.NewTData += this.OnNewTData;
            CloserJoint = closerjoint;
            FurtherJoint = furtherjoint;
            PercentDistance = percentdistnace;
            switch (ft)
            {  
                case FilterType.SGF:
                    {
                        X_Filter = new SavitzkyGolay(7, 4);
                        Y_Filter = new SavitzkyGolay(7, 4);
                        Z_Filter = new SavitzkyGolay(7, 4);
                        break;
                    }
                case FilterType.OurSGF:
                    {
                        X_Filter = new OurSavitzkyGolayFilter(5);
                        Y_Filter = new OurSavitzkyGolayFilter(5);
                        Z_Filter = new OurSavitzkyGolayFilter(5);
                        break;
                    }
                case FilterType.DoubleExponential:
                    {
                        X_Filter = new DoubleExponentialDeriver(alpha, gamma, nd);
                        Y_Filter = new DoubleExponentialDeriver(alpha, gamma, nd);
                        Z_Filter = new DoubleExponentialDeriver(alpha, gamma, nd);
                        break;
                    }
            }
        }

        private VirtualSensorData vsd;
        private void OnNewTData(object sender, KinectData e)
        {
            double[] virtualAcc = new double[] { 0, 0, 0 };
            double[] virtualVel = new double[] { 0, 0, 0 };
            double[] pos = new double[] { e.Joints[FurtherJoint].Position.X, e.Joints[FurtherJoint].Position.Y, e.Joints[FurtherJoint].Position.Z };
            TrackingState trackingstate = e.Body.Joints[this.FurtherJoint].TrackingState;
                //= MathFunctions.midpoint(e.Joints[FurtherJoint], e.Joints[CloserJoint], 100);

            X_Filter.UpdateVal(pos[0], e.NowInTicks);
            Y_Filter.UpdateVal(pos[1], e.NowInTicks);
            Z_Filter.UpdateVal(pos[2], e.NowInTicks);
            
            //acceleration
            var temp = X_Filter.GetNDerivative(nd);
            virtualAcc[0] = temp.Item1;
            virtualAcc[1] = Y_Filter.GetNDerivative(nd).Item1 + 9.81;
            //hack make z -
            virtualAcc[2] = -Z_Filter.GetNDerivative(nd).Item1;

            //position
            double[] virtualPos = new double[] {pos[0], pos[1], pos[2]};

            vsd = new VirtualSensorData();
            vsd.NowInTicks = temp.Item2;
            //vsd.NowInTicks = e.NowInTicks;
            for (int i = 0; i < 3; i++)
            {
                vsd.acceleration[i] = virtualAcc[i];
                vsd.velocity[i] = virtualVel[i];
                vsd.position[i] = virtualPos[i];
            }

            //Rotation
            vsd.rot = e.Body.JointOrientations[JointType.WristRight].Orientation.ToQuaternion(); //Apply filter under this. Note: X,Y,Z,W

            
            //Begin Quat Filter
            #region QuatFilter

            double[] quat = new double[] {vsd.rot.W,vsd.rot.X,vsd.rot.Y,vsd.rot.Z};

            //If this is the first data point, record that. The filter will then not be triggered on the first data point
            //Assuming first data set is correct otherwise all values will be incorrectly flipped. Can improve conditionals later (delay for high confidence data, etc).
            if (DataTracker.vsdFirstQuat[0] == 0 && DataTracker.vsdFirstQuat[1] == 0 && DataTracker.vsdFirstQuat[2] == 0 && DataTracker.vsdFirstQuat[3] == 0)
            { 
                DataTracker.vsdFirstQuat = quat;
                DataTracker.mvsdPrevQuat = quat;
            }

            int counter = 0;

            for (int i = 0; i < 4; i++)
            {
                if (System.Math.Sign(DataTracker.mvsdPrevQuat[i]) != System.Math.Sign(quat[i]))
                    counter++;
            }

            if (counter == 4)
            {
                DataTracker.FlipCounter++;
                for (int j = 0; j < 4; j++)
                    quat[j] = -quat[j];
            }

            //NOTE: just for right wrist, this can be generalized later. Only keep high confidence reference quaternions (except for first data point)
            if (e.Body.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked &&
                e.Body.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked &&
                e.Body.Joints[JointType.WristRight].TrackingState == TrackingState.Tracked)
            {
                for (int i = 0; i < 4; i++)
                    DataTracker.mvsdPrevQuat[i] = quat[i];
            }

            vsd.rot.W = quat[0];
            vsd.rot.X = quat[1];
            vsd.rot.Y = quat[2];
            vsd.rot.Z = quat[3];

            #endregion
            //End Quat Filter
            
            #region HandStateTracking
            if (e.Body.HandLeftState == HandState.Lasso && DataTracker.LastHandState == HandState.Closed)
            {
                if (DataTracker.LassoCount < 2 && Math.Abs(DateTime.Now.Second - DataTracker.PrevSec) < 5)
                    DataTracker.LassoCount++;
                DataTracker.PrevSec = DateTime.Now.Second;
            }

            DataTracker.LastHandState = e.Body.HandLeftState;

            #endregion



            if (this.FurtherJoint == JointType.WristRight)
            { //Only do strict tracking when right wrist is selected
                TrackingState wristState = e.Body.Joints[JointType.WristRight].TrackingState;
                HandState handState = e.Body.HandRightState;
                TrackingState thumbState = e.Body.Joints[JointType.ThumbRight].TrackingState;
                vsd.isinferredornottracked = (wristState != TrackingState.Tracked) || (thumbState != TrackingState.Tracked) || (handState != HandState.Open);
            }
            else
            {
                vsd.isinferredornottracked = (trackingstate == TrackingState.Inferred) || (trackingstate == TrackingState.NotTracked);
            }

            //If in Calibrator Setup (note: change calibrator setup to bool later? - 0 means not in setup) and the data is good, tag it. 
                if (ICherryPicker.isDataGood(vsd.acceleration, !vsd.isinferredornottracked))    //don't need to repeat this in mappedVirtual
                {
                    vsd.section = DataTracker.CurrentSection;
                    DataTracker.ValidVSD = true;
                }
                else
                {
                    vsd.section = 0;
                    DataTracker.ValidVSD = false;
                }
           
        




            if (this.NewIData != null)
                this.NewIData(this, vsd);         //[not edited] This triggers data collection? (So changes must be before this)
            if (this.NewTData != null)
                this.NewTData(this, vsd);

        }


        //public event EventHandler<Exception> OnException;

        public event EventHandler<int> MeasuresPerSec;

        public int MeasuredSamplesPerSecond
        {
            get { throw new NotImplementedException(); }
        }

        public bool IsIRestartable
        {
            get { return (this is IRestartable); }
        }

        public Type IDataType
        {
            get { return typeof(VirtualSensorData); }
        }

        public void Dispose()
        { }
    }
}
