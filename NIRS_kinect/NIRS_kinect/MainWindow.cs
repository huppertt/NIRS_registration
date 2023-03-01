using System;
using Gtk;
using freenect;
using System.Threading;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;
using CloudMapping;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;

public partial class MainWindow : Gtk.Window
{

	Kinect kinect;
	bool[] objectmask;
	Head3D head;

	public MainWindow() : base(Gtk.WindowType.Toplevel)
	{
		Build();
		Gtk.Application.Init();
		Console.WriteLine("There are {0} Kinect Devices Connected", Kinect.DeviceCount);
		head = new Head3D();
		


		if (Kinect.DeviceCount > 0)
		{
			// Connect to first device
			kinect = new Kinect(0);
			if (kinect.IsOpen)
			{
				kinect.Close();
			}

			kinect.Open();
			objectmask = new bool[640 * 480];
			for(int i=0; i< 640*480; i++)
            {
				objectmask[i] = true;
			}
			// Setup event handlers
			kinect.VideoCamera.DataReceived += HandleKinectVideoCameraDataReceived;
			kinect.DepthCamera.DataReceived += HandleKinectDepthCameraDataReceived;
			DepthFrameMode[] frameModes = kinect.DepthCamera.Modes;
			VideoFrameMode[] frameModes1 = kinect.VideoCamera.Modes;
			kinect.VideoCamera.Mode = frameModes1[1];
			kinect.DepthCamera.Mode = frameModes[4];


			int h = 640;
			int w = 480;
			var bmp = new Bitmap(w, h, PixelFormat.Format32bppArgb);

			for (int i = 0; i < h; i++)
			{
				for (int j = 0; j < w; j++)
				{
					bmp.SetPixel(j, i, Color.Black);
					
				}
			}
			MemoryStream ms = new MemoryStream();
			bmp.Save(ms, ImageFormat.Png);
			ms.Position = 0;

			Gdk.Pixbuf pixbuf = new Gdk.Pixbuf(ms);
			image4.Pixbuf = pixbuf.ScaleSimple(640, 480, Gdk.InterpType.Bilinear);
			image4.QueueDraw();
			image5.Pixbuf = pixbuf.ScaleSimple(640, 480, Gdk.InterpType.Bilinear);
			image5.QueueDraw();



			// Set LED to Yellow
			kinect.LED.Color = LEDColor.BlinkRedYellow;

			// Set tilt to halfway up
			kinect.Motor.Tilt = 0.5;
			// Start update thread
			Thread t = new Thread(new ThreadStart(delegate ()
			{
				while (true)
				{
					// Update status of accelerometer/motor etc.
					kinect.UpdateStatus();

					// Process any pending events.
					Kinect.ProcessEvents();
					Thread.Sleep(10);
				}
			}));
			t.Start();
		}
	}

		/// <summary>
		/// Handle depth data
		/// </summary>
		/// <param name="sender">
		/// A <see cref="System.Object"/>
		/// </param>
		/// <param name="e">
		/// A <see cref="BaseCamera.DataReceivedEventArgs"/>
		/// </param>
		private void HandleKinectDepthCameraDataReceived(object sender, BaseCamera.DataReceivedEventArgs e)
		{

		List<Vector<double>> newdata = new List<Vector<double>>();

		Console.WriteLine("Depth data received at {0}", e.Timestamp);
		byte[] pixels = e.Data.Data;
		int w = e.Data.Width;
		int h = e.Data.Height;
		var bmp = new Bitmap(w, h, PixelFormat.Format32bppArgb);

		double pmin = 255;
		double pmax = 0;
		double smin = hscrollbar1.Value/100;
		double smax = hscrollbar2.Value/100;
		double[] pix = new double[w * h];
		for(int i=0; i<w*h; i++)
        {
			
			pix[i] = (double)pixels[(i*2)] + 255*(double)pixels[(i * 2) + 1];
            if (pix[i] > pmax)
            {
				pmax = pix[i];
            }
            if (pix[i] < pmin)
            {
				pmin = pix[i];
            }
        }



		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				int p = (w * i + j);

				double v = pix[p]/pmax;
				objectmask[(w * i + j)] = (v > smin) & (v < smax);

				if (objectmask[(w * i + j)])
				{
					newdata.Add(Vector<double>.Build.Dense(new double[] { i, j, pix[p] }));
					bmp.SetPixel(j, i, Color.FromArgb(255, (int)(v * 255), (int)(v * 255), (int)(v * 255)));
                }
                else
                {
					bmp.SetPixel(j, i, Color.Black);

				}


			}
		}
		if (checkbutton1.Active)
		{
			head.AddPoints(newdata);
		}
		MemoryStream ms = new MemoryStream();
		bmp.Save(ms, ImageFormat.Png);
		ms.Position = 0;

		image4.Pixbuf = new Gdk.Pixbuf(ms);
		image4.QueueDraw();

	}



		/// <summary>
		/// Handle video data
		/// </summary>
		/// <param name="sender">
		/// A <see cref="System.Object"/>
		/// </param>
		/// <param name="e">
		/// A <see cref="BaseCamera.DataReceivedEventArgs"/>
		/// </param>
		private void HandleKinectVideoCameraDataReceived(object sender, BaseCamera.DataReceivedEventArgs e)
		{
		Console.WriteLine("Video data received at {0}", e.Timestamp);
		byte[] pixels = e.Data.Data;
		int w = e.Data.Width;
		int h = e.Data.Height;
		var bmp = new Bitmap(w, h, PixelFormat.Format32bppArgb);

		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				if (objectmask[(w * i + j)])
				{
					int p = 3 * (w * i + j);
					bmp.SetPixel(j, i, Color.FromArgb(255, pixels[p +
					0], pixels[p + 1], pixels[p + 2]));
                }
                else
                {
					bmp.SetPixel(j, i, Color.Black);
				}
			}
		}
		MemoryStream ms = new MemoryStream();
		bmp.Save(ms, ImageFormat.Png);
		ms.Position = 0;

		Gdk.Pixbuf pixbuf = new Gdk.Pixbuf(ms);
		image5.Pixbuf=pixbuf.ScaleSimple(640, 480, Gdk.InterpType.Bilinear);
		image5.QueueDraw();

	}


	

	protected void OnDeleteEvent(object sender, DeleteEventArgs a)
		{
		kinect.LED.Color = LEDColor.None;
		kinect.Motor.Tilt = 0;

		// Set tilt to halfway up
		kinect.Close();
			Application.Quit();
			a.RetVal = true;
		}

    protected void ChangeMinDepth(object sender, EventArgs e)
    {
		spinbutton1.Value = hscrollbar1.Value;
    }

    protected void ChangeMaxDepth(object sender, EventArgs e)
    {
		spinbutton2.Value = hscrollbar2.Value;
	}

    protected void ChangeMinSpin(object sender, EventArgs e)
    {
		hscrollbar1.Value= spinbutton1.Value;

	}

    protected void CHnageMaxSpin(object sender, EventArgs e)
    {
		hscrollbar2.Value = spinbutton2.Value;
	}

    protected void Start(object sender, EventArgs e)
    {
		// Start cameras
		if (!kinect.VideoCamera.IsRunning)
		{
			kinect.VideoCamera.Start();
			kinect.DepthCamera.Start();
		}

	}

	protected void Stop(object sender, EventArgs e)
    {
		// Start cameras\
		if (kinect.VideoCamera.IsRunning)
		{
			kinect.VideoCamera.Stop();
			kinect.DepthCamera.Stop();
		}
	}
}
