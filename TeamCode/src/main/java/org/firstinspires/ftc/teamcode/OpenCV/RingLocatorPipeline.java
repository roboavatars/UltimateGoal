package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class RingLocatorPipeline extends OpenCvPipeline {

    // CV Thresholds
    public static double FILTER_MIN = 80;
    public static double FILTER_MAX = 115;
    public static double HEIGHT_THRESH = 3;
    public static double WIDTH_THRESH = 15;

    // Perspective Math Constants
    public static double CAM_HEIGHT = 8.5;
    public static double CAM_FOV = Math.toRadians(15);
    public static double CAM_THETA = Math.toRadians(57);

    // Image Processing Mats
    private Mat processed = new Mat();
    private Mat mask = new Mat();

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
    }

    @Override
    public Mat processFrame(Mat input) {
        // Crop Image
        Rect rect = new Rect(5, 10, 140, 100);
        processed = new Mat(input, rect);

        // Convert to YCrCb Color Space
        Imgproc.cvtColor(input, processed, Imgproc.COLOR_RGB2YCrCb);

        // Extract Cb
        Core.extractChannel(processed, processed, 2);

        // Filter Colors
        Core.inRange(processed, new Scalar(FILTER_MIN), new Scalar(FILTER_MAX), processed);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

        // Mask Image for Debugging
        input.copyTo(mask, processed);

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Save Images for Debug
        saveMatToDisk(input, "input");
        saveMatToDisk(processed, "processed");
        saveMatToDisk(mask, "mask");

        // Loop Through Contours
        distMin = Double.MAX_VALUE;
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
        if (distMin == Double.MAX_VALUE) {
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
        double num = (Math.cos(-CAM_FOV - theta)) / (2 * Math.sin(theta)) + yPix * Math.sin(CAM_FOV);
        double den = (Math.sin(-CAM_FOV - theta)) / (2 * Math.sin(theta)) + yPix * Math.cos(CAM_FOV);
        return -CAM_HEIGHT * num / den;
    }

    public double[] getAbsRingPos(double robotX, double robotY, double robotTheta) {
        double ringX = ringPos[0];
        double ringY = ringPos[1];

        double targetX = robotX + ringX * Math.sin(robotTheta) + ringY * Math.cos(robotTheta);
        double targetY = robotY + ringX * Math.cos(robotTheta) + ringY * Math.sin(robotTheta);
        double targetTheta = Math.atan2(targetY - robotY, targetX - robotX);

        return new double[] {targetX, targetY, targetTheta};
    }

    public double[] getRelRingPos() {
        return ringPos;
    }

    private void log(String message) {
        Log.w("ring-locator-pipe", message);
    }
}