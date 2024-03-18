package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;


public class USBCameraVision extends SubsystemBase {
    Thread visionThread;
    UsbCamera camera;

    public USBCameraVision() {
        visionThread = new Thread(() -> {
            // Get the Axis camera from CameraServer
            camera = CameraServer.startAutomaticCapture();
            // Set the resolution
            camera.setResolution(160, 120);

//             // Get a CvSink. This will capture Mats from the camera
//             CvSink cvSink = CameraServer.getVideo();
//             // Set up a CvSource. This will send images back to the Dashboard
//             CvSource outputStream = CameraServer.putVideo("Driver Camera", 480, 640);

//             Mat mat = new Mat();

//             while (!Thread.interrupted()) {
//                 // Tell the CvSink to grab a frame from the camera and put it
//                 // in the source mat. If there is an error notify the output.
//                 if (cvSink.grabFrame(mat) == 0) {
//                     // Send the output the error.
//                     outputStream.notifyError(cvSink.getError());
//                     // skip the rest of the current iteration
//                     continue;
//                 }
// //                Core.transpose(mat, mat);
//                 Core.flip(mat, mat, 0);
//                 //Imgproc.blur(mat, mat, Size());
//                 outputStream.putFrame(mat);
//             }
        });
        visionThread.setDaemon(true);
    }

    public void start() {
        visionThread.start();
    }

    public void addCameraToDrivingTab(ShuffleboardTab drivingTab) {
        try {
            drivingTab.add("Driver Camera", camera).
                    withPosition(4,0).withSize(2,3).
                    withProperties(
                            Map.of("showCrosshair", false, "showControls", false, "rotation", "HALF", "showGlyph", false, "title", "Driver Camera")
                    );
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
}