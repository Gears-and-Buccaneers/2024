package frc.system;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

public class Vision {
    public Vision() {
        Thread visionThread = new Thread(() -> {
            // Get the UsbCamera from CameraServer
            CameraServer.startAutomaticCapture();

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();

            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat1 = new Mat();
            Mat mat2 = new Mat();

            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat1) == 0) {
                    // Send the output the error.
                    outputStream.notifyError(cvSink.getError());
                    // skip the rest of the current iteration
                    continue;
                }

                Core.inRange(mat1, new Scalar(245, 245, 245), new Scalar(255, 255, 255), mat2);

                // Give the output stream a new image to display
                outputStream.putFrame(mat2);
            }
        });

        visionThread.setDaemon(true);
        visionThread.start();
    }
}
