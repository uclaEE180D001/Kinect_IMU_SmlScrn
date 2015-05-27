namespace BluetoothController.UserControls.WinFromUserContorls
{
    partial class CalibratorControl
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.timerContainer = new System.ComponentModel.Container();
            this.SetupButton = new System.Windows.Forms.Button();
            this.CalibrateButton = new System.Windows.Forms.Button();
            this.OutputTextBox = new System.Windows.Forms.TextBox();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.DataProgressBar = new BluetoothController.UserControls.WinFromUserContorls.CalibratorProgressBar();
            this.SetupGesture = new System.Windows.Forms.Timer(this.timerContainer);
            this.SuspendLayout();
            // 
            // SetupButton
            // 
            this.SetupButton.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.SetupButton.Location = new System.Drawing.Point(0, 0);
            this.SetupButton.Name = "SetupButton";
            this.SetupButton.Size = new System.Drawing.Size(305, 23);
            this.SetupButton.TabIndex = 0;
            this.SetupButton.Text = "Setup Calibrator";
            this.SetupButton.UseVisualStyleBackColor = true;
            this.SetupButton.Click += new System.EventHandler(this.SetupButton_Click);
            //
            // SetupGesture
            //
            this.SetupGesture.Enabled = true;
            this.SetupGesture.Tick += new System.EventHandler(this.Setup_Gesture);

            // 
            // CalibrateButton
            // 
            this.CalibrateButton.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.CalibrateButton.Enabled = false;
            this.CalibrateButton.Location = new System.Drawing.Point(0, 58);
            this.CalibrateButton.Name = "CalibrateButton";
            this.CalibrateButton.Size = new System.Drawing.Size(305, 23);
            this.CalibrateButton.TabIndex = 2;
            this.CalibrateButton.Text = "Calibrate";
            this.CalibrateButton.UseVisualStyleBackColor = true;
            this.CalibrateButton.Click += new System.EventHandler(this.CalibrateButton_Click);
            // 
            // OutputTextBox
            // 
            this.OutputTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.OutputTextBox.Enabled = false;
            this.OutputTextBox.Location = new System.Drawing.Point(0, 88);
            this.OutputTextBox.Multiline = true;
            this.OutputTextBox.Name = "OutputTextBox";
            this.OutputTextBox.ReadOnly = true;
            this.OutputTextBox.Size = new System.Drawing.Size(304, 126);
            this.OutputTextBox.TabIndex = 3;
            this.OutputTextBox.Text = "Predicted linear/rotational difference:";
            this.OutputTextBox.TextChanged += new System.EventHandler(this.OutputTextBox_TextChanged);
            // 
            // textBox1
            // 
            this.textBox1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.textBox1.Enabled = false;
            this.textBox1.Location = new System.Drawing.Point(1, 252);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.Size = new System.Drawing.Size(301, 20);
            this.textBox1.TabIndex = 4;
            this.textBox1.Text = "Filtered Frames: ";
            this.textBox1.TextChanged += new System.EventHandler(this.textBox1_TextChanged);
            // 
            // DataProgressBar
            // 
            this.DataProgressBar.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.DataProgressBar.ForeColor = System.Drawing.Color.SpringGreen;
            this.DataProgressBar.Location = new System.Drawing.Point(1, 29);
            this.DataProgressBar.MarqueeAnimationSpeed = 25;
            this.DataProgressBar.Name = "DataProgressBar";
            this.DataProgressBar.Size = new System.Drawing.Size(303, 23);
            this.DataProgressBar.Step = 2;
            this.DataProgressBar.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.DataProgressBar.TabIndex = 1;
            // 
            // CalibratorControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.OutputTextBox);
            this.Controls.Add(this.CalibrateButton);
            this.Controls.Add(this.DataProgressBar);
            this.Controls.Add(this.SetupButton);
            this.Name = "CalibratorControl";
            this.Size = new System.Drawing.Size(305, 275);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button SetupButton;
        private BluetoothController.UserControls.WinFromUserContorls.CalibratorProgressBar DataProgressBar;
        private System.Windows.Forms.Button CalibrateButton;
        private System.Windows.Forms.TextBox OutputTextBox;
        private System.Windows.Forms.TextBox textBox1;
        private System.ComponentModel.IContainer timerContainer;
        private System.Windows.Forms.Timer SetupGesture;
    }
}
