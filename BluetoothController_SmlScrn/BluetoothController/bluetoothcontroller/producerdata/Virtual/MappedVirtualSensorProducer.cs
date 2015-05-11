﻿using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BluetoothController.Kinect;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;
using System.Threading;

namespace BluetoothController
{
    public class MappedVirtualSensorProducer : IDataProducer<VirtualSensorData>, IDataProducer
    {
        private BlockingCollection<KinectData> KinectFrameBlockingCollection = new BlockingCollection<KinectData>();
        private ConcurrentQueue<VirtualSensorData> VirtualSensorDataConQueue = new ConcurrentQueue<VirtualSensorData>();
        private IDataProducer<KinectData> KDDataProducer;
        private IDataProducer<VirtualSensorData> VSDDataProducer;
        private JointType JT;
        private VirtualSensorData Data;

        public MappedVirtualSensorProducer(IDataProducer<KinectData> idataproducer_kinectdata, IDataProducer<VirtualSensorData> idataproducer_virtualsensor, JointType jt = JointType.WristRight)
        {
            this.KDDataProducer = idataproducer_kinectdata;
            this.VSDDataProducer = idataproducer_virtualsensor;
            this.JT = jt;
            this.VSDDataProducer.NewTData += this.OnVSDNewTData;
            this.KDDataProducer.NewTData += this.OnKDNewTData;
            Task.Factory.StartNew(this.Worker, TaskCreationOptions.LongRunning);
        }




        void Worker()
        {
            try
            {
                //Thread.Sleep(1000);
                VirtualSensorData FrontVSD, NextVSD = null;
                bool TryPeekGood = false;
                QuaternionCoordinateMapper Mapper = new QuaternionCoordinateMapper() { CoefficientVector = new Vector3D(1.0, 1.0, 1.0), Initial = new Quaternion(0,0,0,1) };
                foreach (KinectData kd in this.KinectFrameBlockingCollection.GetConsumingEnumerable())
                {
                    while (this.VirtualSensorDataConQueue.Count < 3)
                        Thread.Sleep(20);
                    TryPeekGood = this.VirtualSensorDataConQueue.TryDequeue(out FrontVSD);
                    TryPeekGood = TryPeekGood && this.VirtualSensorDataConQueue.TryPeek(out NextVSD);
                    while (kd.NowInTicks >= NextVSD.NowInTicks)
                    {
                        if (this.VirtualSensorDataConQueue.Count > 1)
                        {
                            this.VirtualSensorDataConQueue.TryDequeue(out FrontVSD);
                            this.VirtualSensorDataConQueue.TryPeek(out NextVSD);
                        }
                        else
                            Thread.SpinWait(30);
                    }

                    //Here is where I will do the conversion.
                    this.Data = new VirtualSensorData(FrontVSD);


                    //Mapper.Initial = kd.Body.JointOrientations[JT].Orientation.ToQuaternion();
                    Quaternion SensorInverse = kd.Body.JointOrientations[this.JT].Orientation.ToQuaternion();
                    
                    //Begin Quat Filter
                    #region QuatFilter

                    double[] quat = new double[] { SensorInverse.W, SensorInverse.X, SensorInverse.Y, SensorInverse.Z };

                    //If this is the first data point, record that. The filter will then not be triggered on the first data point
                    //Assuming first data set is correct otherwise all values will be incorrectly flipped. Can improve conditionals later (delay for high confidence data, etc).
                    if (DataTracker.mvsdFirstQuat[0] == 0 && DataTracker.mvsdFirstQuat[1] == 0 && DataTracker.mvsdFirstQuat[2] == 0 && DataTracker.mvsdFirstQuat[3] == 0)
                    {
                        DataTracker.mvsdFirstQuat = quat;
                        DataTracker.mvsdPrevQuat = quat;
                    }

                    int counter = 0;

                    for (int i = 0; i < 4; i++)
                    {
                        if (System.Math.Sign(DataTracker.mvsdPrevQuat[i]) != System.Math.Sign(quat[i]) )
                            counter++;
                    }

                    if (counter == 4)
                    {
                        DataTracker.FlipCounter++;
                        for (int j = 0; j < 4; j++)
                            quat[j] = -quat[j];
                    }
                    // NOTE: MAY want to either make this just an else statement, or include that the thumb must be tracked as well
                        //NOTE: just for right wrist, this can be generalized later. Only keep high confidence reference quaternions (except for first data point)
                    if (kd.Body.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked &&
                        kd.Body.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked &&
                        kd.Body.Joints[JointType.WristRight].TrackingState == TrackingState.Tracked &&
                        kd.Body.Joints[JointType.ThumbRight].TrackingState == TrackingState.Tracked)
                    {
                        for (int i = 0; i < 4; i++)
                            DataTracker.mvsdPrevQuat[i] = quat[i];
                    }

                    SensorInverse.W = quat[0];
                    SensorInverse.X = quat[1];
                    SensorInverse.Y = quat[2];
                    SensorInverse.Z = quat[3];
                    
                    #endregion
                    
                    //End Rotation Filter


                    SensorInverse.Invert();
                    Vector3D temvec = Mapper.Map(SensorInverse, FrontVSD.acceleration.ToVector());
                    //hack -z
                    Vector3D corvec = new Vector3D(-temvec.Z, temvec.Y, temvec.X);
                    //this.Data.acceleration = Mapper.Map(SensorInverse, FrontVSD.acceleration.ToVector()).ToArray();
                    this.Data.acceleration = corvec.ToArray();
                    if (this.NewIData != null)
                        this.NewIData(this, this.Data);
                    if (this.NewTData != null)
                        this.NewTData(this, this.Data);

                }
            }
            catch (Exception e)
            {
                throw e;
            }
        }


        void OnKDNewTData(object sender, KinectData e)
        {
            //if this isn't TRYadd then it will lock becuase the KD depends on VSD and VSD is fired after KD
            this.KinectFrameBlockingCollection.TryAdd(e);
        }
        void OnVSDNewTData(object sender, VirtualSensorData e)
        {
            this.VirtualSensorDataConQueue.Enqueue(e);
        }



        #region IDataProducer<VirtualSensorData> Members

        public event EventHandler<VirtualSensorData> NewTData;

        #endregion

        #region IDataProducer Members

        public event EventHandler<IData> NewIData;

        public event EventHandler<int> MeasuresPerSec;

        public SensorType SensorType
        {
            get { return BluetoothController.SensorType.MappedVirtual; }
        }

        public Type IDataType
        {
            get { return typeof(KinectData); }
        }

        public string DeviceName
        {
            get { return "Mapped_" + this.VSDDataProducer.DeviceName; }
        }

        public string DeviceAddress
        {
            get { return "Mapped_" + this.VSDDataProducer.DeviceAddress; }
        }

        public bool IsIRestartable
        {
            get { return (this is IRestartable); }
        }

        #endregion

        #region IDisposable Members

        public void Dispose()
        {
            //TODO: Add dispose to MappedVirtualSensorProducer
        }

        #endregion
    }
}
