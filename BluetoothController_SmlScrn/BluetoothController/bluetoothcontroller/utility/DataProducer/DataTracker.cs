//use to apply anything that relies on past/future data points (filter and session ID) [maybe use something similar for gesture control?]
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows.Media.Media3D;

namespace BluetoothController
{
    public static class DataTracker
    {
        public static Quaternion prevRot;   //for filter
        public static int CurrentSection;   //Gets called in constructor. =0 when not in 'calibration setup'. = SessionCounter when in 'calibration setup'
        public static int SectionCounter;   //Keeps track of # of times Calibration Setup has been called
        public static double[] vsdFirstQuat = new double[4] { 0, 0, 0, 0 };
        public static double[] vsdPrevQuat = new double[4] { 0, 0, 0, 0 };
        public static double[] mvsdFirstQuat = new double[4] { 0, 0, 0, 0 };
        public static double[] mvsdPrevQuat = new double[4] { 0, 0, 0, 0 };
        public static int FlipCounter;  //tracks the number of times a sign flip has been applied to the data
        public static bool ValidVSD;    //True if Calibrator Setup is on AND the VSD is considered 'good'
        static DataTracker()
        {
            CurrentSection = 0;
            SectionCounter = 0;
            FlipCounter = 0;
            ValidVSD = false;
        }
    }
}

//Note: may want to add "calibrator setup on" bool and split validVSD into 2 bools (1 for valid data and another for whether setup is on... right now the one bool is only true for both being true)