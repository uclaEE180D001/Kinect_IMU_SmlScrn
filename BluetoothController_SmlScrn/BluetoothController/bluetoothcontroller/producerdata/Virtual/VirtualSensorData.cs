﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace BluetoothController
{
    public class VirtualSensorData : IData
    {
        public double[] acceleration;
        public double[] velocity;
        public double[] position;
        public int section; //0 means not used for calibration, # indicates which calibration section/interval this value falls in
        public Quaternion rot;
        public bool isinferredornottracked;
        public long NowInTicks { get;  set; }
        public VirtualSensorData()
        {
            this.acceleration = new double[3];
            this.velocity = new double[3];
            this.position = new double[3];
            this.isinferredornottracked = false;
            this.NowInTicks = DateTime.UtcNow.Ticks;
            this.section = 0;
        }
        public VirtualSensorData(VirtualSensorData v)
        {
            this.isinferredornottracked = v.isinferredornottracked;
            this.NowInTicks = v.NowInTicks;

            this.acceleration = new double[v.acceleration.Length];
            for (int i = 0; i < v.acceleration.Length; i++)
                this.acceleration[i] = v.acceleration[i];

            this.velocity = new double[v.velocity.Length];
            for (int i = 0; i < v.velocity.Length; i++)
                this.velocity[i] = v.velocity[i];

            this.position = new double[v.position.Length];
            for (int i = 0; i < v.position.Length; i++)
                this.position[i] = v.position[i];

            this.rot = v.rot;

            this.section = v.section;
        }

        #region IData Members
        public string ToFileHeaderString()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("{0}, {1}, {2}, {3}, {4}{5}",
                "Time",
                "Ax",
                "Ay",
                "Az",
                "sID",
                Environment.NewLine);
            return sb.ToString();
        }
        public string ToFileString()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("{0}, {1}, {2}, {3}, {4}",
                this.NowInTicks,
                this.acceleration[0],
                this.acceleration[1],
                this.acceleration[2],
                this.section);
            sb.Append(Environment.NewLine);
            return sb.ToString();
        }
        public string ToPreviewString()     // 4-17-15 Need to Change for section ID?
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("vx:{0:D},vy:{1:D},vz{2:D}",
                velocity[0].ToString("+#.###;-#.###;0.000"),
                velocity[1].ToString("+#.###;-#.###;0.000"),
                velocity[2].ToString("+#.###;-#.###;0.000"));
            sb.AppendFormat("ax:{0:D},ay:{1:D},az{2:D}",
                acceleration[0].ToString("f"),
                acceleration[1].ToString("f"),
                acceleration[2].ToString("f"));
            return sb.ToString();
        }
        public string ToWindowsFormString()
        {
            throw new NotImplementedException();
        }
        #endregion
    }
}
