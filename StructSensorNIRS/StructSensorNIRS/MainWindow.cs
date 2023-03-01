using System;
using Gtk;
using Nimble;
using Nimble.Native;
using System.Linq;
using System.Windows.Media.Imaging;
using System.Windows.Media;

public partial class MainWindow : Gtk.Window
{
    //private readonly WriteableBitmap _bitmap;
    Device _device;
    VideoStream _stream;
    OpenNI _openni;
    Image _image;

    private readonly WriteableBitmap _bitmap;

    public MainWindow() : base(Gtk.WindowType.Toplevel)
    {
        Build();

        _image = new Image();
        var vbox = new VBox(false, 1);
        
       



        // swScroll.AddWithViewport(picBox);
        vbox.PackStart(_image, true, true, 1);

        this.Add(vbox);
        this.ShowAll();
        
       
        _openni = new OpenNI();
        _openni.Initialize();

        var devices = _openni.Devices;
        foreach (var d in devices)
        {
            System.Console.WriteLine(d.Name);
        }
        var deviceInfo = _openni.Devices.First();
        _device = deviceInfo.Open();
    
        _stream = _device.OpenDepthStream();
        _stream.Start();

        _device.ImageRegistration = true ? ImageRegistrationMode.DepthToColor : ImageRegistrationMode.None;
        _stream.NewFrame += _stream_NewFrame;

        
        
    }

    protected void OnDeleteEvent(object sender, DeleteEventArgs a)
    {
        _stream.Close();
        _openni.Shutdown();
        Application.Quit();
        a.RetVal = true;
    }

    void _stream_NewFrame(VideoStream stream, Nimble.Frame frame)
    {
        var rect = new Int32Rect(0, 0, frame.Width, frame.Height);
        using (var pixels = frame.LockPixels())
        {
            _image.Pixbuf.
            _bitmap.WritePixels(rect, pixels.Data, pixels.DataSize, pixels.Stride);
        }
        frame.Dispose();
        return;
        
    }

}
