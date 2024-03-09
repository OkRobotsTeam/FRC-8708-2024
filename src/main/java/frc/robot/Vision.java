package frc.robot;

import edu.wpi.first.util.PixelFormat;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Vision extends Thread {
    UsbCamera camera;

    public void run() {
        System.out.println("Connecting to camera");
        camera = new UsbCamera("Webcam1", 0);
        camera.setVideoMode(PixelFormat.kYUYV, 320, 240, 6);
        System.out.println("Starting frame capture on camera");
        MjpegServer server = CameraServer.startAutomaticCapture(camera);


        // Reduce the impact on bandwidth
        server.setCompression(50);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo(camera);

        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Camera", 320, 240);

        outputStream.setPixelFormat(PixelFormat.kMJPEG);
        Shuffleboard.getTab("Driving").add(outputStream).withPosition(0, 0).withSize(4, 4);
        Shuffleboard.update();

        Mat mat = new Mat();

        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                continue;
            } else {
                outputStream.putFrame(mat);
            }
        }
    }
}