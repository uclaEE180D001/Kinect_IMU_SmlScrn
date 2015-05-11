using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BluetoothController
{
    public static class ICherryPicker
    {
        public const double MaxAcceleration = 15.0;
        private static int filteredFrames = 0;

        public static int GetFilteredFrames()
        {
            return filteredFrames;
        }

        public static void ResetFilteredFrames()
        {
            filteredFrames = 0;
        }

        public static bool isDataWithinBounds(double[] data)
        {
            bool IsGood = true;
            if (Math.Abs(data[0]) > MaxAcceleration)
                IsGood = false;
            if (Math.Abs(data[1]) > MaxAcceleration) 
                IsGood = false;
            if (Math.Abs(data[2]) > MaxAcceleration)
                IsGood = false;
            return IsGood;
        }

        //Returning false means the supplied data should be filtered.
        //Returning true means the data should be kept.
        public static bool isDataGood(double[] data, bool isTracked)
        {
            if ((!isTracked) || (!isDataWithinBounds(data)))
            {
                filteredFrames++;
                return false;
            }
            else
            {
                return true;
            }
        }
    }
}
