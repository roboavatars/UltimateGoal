package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class RingLocatorPipeline extends OpenCvPipeline {

    // CV Thresholds
    public static double HEIGHT_THRESH = 3;
    public static double WIDTH_THRESH = 15;

    // Perspective Math Constants
    public static double CAM_HEIGHT = 10;
    public static double CAM_THETA = Math.toRadians(15);
    public static double CAM_VFOV = Math.toRadians(47);

    // Image Processing Mats
    private RingProcessor processor;
    private Mat[] processorOut = new Mat[3];
    private Mat processed = new Mat();

    // Ellipse Variables
    private double width;
    private double height;
    private double xPix;
    private double yPix;

    // Ring Position Variables
    private double xRel;
    private double yRel;
    private double dist;
    private double distMin;
    private double[] ringPos = new double[2];

    public RingLocatorPipeline() {
        // Clear Old Images
        @SuppressLint("SdCardPath") File dir = new File("/sdcard/EasyOpenCV/ringLocator");
        String[] children = dir.list();
        if (children != null) {
            for (String child : children) {
                new File(dir, child).delete();
            }
        }

        processor = new RingProcessor();
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process Image
        processorOut = processor.processFrame(input);
        input = processorOut[0];
        processed = processorOut[1];

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop Through Contours
        distMin = 48 * Math.sqrt(13); // Longest Diagonal on Field
        for (MatOfPoint contour : contours) {

            // Bound Ellipse
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect approxEllipse = Imgproc.fitEllipse(areaPoints);

            // Save Ellipse Data
            width = approxEllipse.size.width;
            height = approxEllipse.size.height;
            xPix = (approxEllipse.center.x - 160) / 320; // x ∈ [-1, 1]
            yPix = approxEllipse.center.y / 240; // y ∈ (0, 1)

            // Analyze Valid Contours
            if (height > HEIGHT_THRESH && width > WIDTH_THRESH) {

                // Calculate Center of Ring if Y is in Valid Domain
                if (0 < yPix && yPix < 0.7468) {
                    xRel = calcX(xPix, yPix);
                    yRel = calcY(yPix);
                    dist = Math.hypot(xRel, yRel);

                    log("(" + xRel + ", " + yRel + ") " + dist);

                    // Save Closest Ring Position to Robot
                    if (yRel > 0 && dist < distMin) {
                        distMin = dist;
                        ringPos = new double[] {xRel, yRel};
                    }
                }
            }
        }

        // Return (-1, -1) If No Rings Detected
        if (distMin == 48 * Math.sqrt(13)) {
            ringPos = new double[] {-1, -1};
            log("No Rings Detected");
        }

        log("Result: " + Arrays.toString(ringPos));

        return input;
    }

    private double calcX(double xPix, double yPix) {
        double theta = CAM_THETA / 2;
        return Math.tan(theta) * xPix * Math.hypot(CAM_HEIGHT, calcY(yPix));
    }

    private double calcY(double yPix) {
        double theta = CAM_THETA / 2;
        double num = (Math.cos(-CAM_VFOV - theta)) / (2 * Math.sin(theta)) + yPix * Math.sin(CAM_VFOV);
        double den = (Math.sin(-CAM_VFOV - theta)) / (2 * Math.sin(theta)) + yPix * Math.cos(CAM_VFOV);
        return -CAM_HEIGHT * num / den;
    }

    public double[] getAbsRingPos(double robotX, double robotY, double robotTheta) {
        // Calculate ring's absolute position using robot's position and ring's position relative to robot
        double ringX = ringPos[0];
        double ringY = ringPos[1];

        double targetX = robotX + ringX * Math.sin(robotTheta) + ringY * Math.cos(robotTheta);
        double targetY = robotY + ringX * Math.cos(robotTheta) + ringY * Math.sin(robotTheta);

        return new double[] {targetX, targetY};
    }

    public double[] getRelRingPos() {
        return ringPos;
    }

    private void log(String message) {
        Log.w("ring-locator-pipe", message);
    }
}