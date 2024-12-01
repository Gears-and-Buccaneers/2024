package frc.system;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj2.command.Command;

public class Vision extends Thread {
    final Target[] targets;

    // Whether the vision executor should be active.
    boolean active = false;

    final CvSink source = CameraServer.getVideo();
    final CvSource stream = CameraServer.putVideo("Rectangle", 640, 480);

    public static class Target {
        // Top-left point
        public double x1;
        public double y1;
        // Bottom-right point
        public double x2;
        public double y2;
        // Detection threshold
        public int threshold;
        // The command to run during detection
        public Command cmd;

        public Target(double x1, double y1, double x2, double y2, int threshold, Command cmd) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
            this.threshold = threshold;
            this.cmd = cmd;
        }
    }

    public Vision(Target targets[]) {
        this.targets = targets;

        setDaemon(true);

        CameraServer.startAutomaticCapture();

        start();
    }

    @Override
    public void run() {
        Mat mat = new Mat();

        // Main loop of the thread.
        while (true) {
            // Acquire the lock on the vision state.
            synchronized (this) {
                while (true) {
                    // Check if the thread should continue.
                    if (active) {
                        // If so, then execute another iteration of the vision loop.
                        break;
                    }

                    try {
                        // Otherwise, wait for a ping from `start`, relinquishing the lock to allow the
                        // active state to change.
                        // Then, continue, and check the active state again.
                        wait();
                    } catch (InterruptedException e) {
                        return;
                    }
                }
            }

            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (source.grabFrame(mat) == 0) {
                // Send the output the error.
                stream.notifyError(source.getError());
                // skip the rest of the current iteration
                continue;
            }

            int c = mat.cols();
            int r = mat.rows();
            
            for(Target t : targets) {
                Rect rect = new Rect(
                    (int)(t.x1 * c),
                    (int)(t.y1 * r),
                    (int)(t.x2 * c),
                    (int)(t.y2 * r)
                );

                // Slice the mat to the target rectangle
                Mat submat = mat.submat(rect);
                // Find the average pixel value
                double[] mean = Core.mean(submat).val;
                // Check if the threshold is met
                boolean detected = mean[0] > t.threshold && mean[1] > t.threshold && mean[2] > t.threshold;

                if (detected) t.cmd.schedule();
                else t.cmd.cancel();

                Imgproc.rectangle(mat, rect, new Scalar(0, detected ? 255 : 0, detected ? 0 : 255), 5);
            }

            // Give the output stream a new image to display
            stream.putFrame(mat);
        }
    }

    public synchronized void notifyStart() {
        // Set the active flag.
        active = true;
        // Notify the thread. When this function exits, the lock on the object will be
        // relinquished, resuming the thread.
        notify();
    }

    public synchronized void notifyStop() {
        // Set the active flag. The thread will view this flag and enter the waiting
        // state.
        active = false;
    }
}
