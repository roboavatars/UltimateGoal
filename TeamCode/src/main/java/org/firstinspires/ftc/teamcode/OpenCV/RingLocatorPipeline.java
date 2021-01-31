package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
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
    public static double HEIGHT_THRESH = 15;
    public static double WIDTH_THRESH = 3;

    // Camera Constants
    public static double CAM_HEIGHT = 8;
    public static double CAM_FRONT = 7; // y distance between camera and robot center
    public static double CAM_PHI = Math.toRadians(12.5);
    public static double CAM_VFOV = Math.toRadians(43);
    public static double CAM_HFOV = Math.toRadians(60);

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
            Point[] contourArray = contour.toArray();

            // Bound Ellipse if Contour is Large Enough
            if (contourArray.length >= 5) {
                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                RotatedRect ellipse = Imgproc.fitEllipse(areaPoints);

                // Save Ellipse Data
                width = ellipse.size.width;
                height = ellipse.size.height;
                xPix = ellipse.center.x / 160 - 1; // x ∈ [-1, 1]
                yPix = 1 - ellipse.center.y / 180; // y ∈ (0, 1)

//                log("x: " + xPix);
//                log("y: " + yPix);
//                log("width: " + width);
//                log("height: " + height);

                // Analyze Valid Contours
                if (height > HEIGHT_THRESH && width > WIDTH_THRESH) {
                    Imgproc.ellipse(input, ellipse, new Scalar(0, 255, 0), 1);

                    // Calculate Center of Ring if Y is in Valid Domain
                    if (0 < yPix && yPix < 0.8827) {
                        double[] xy = map2Dto3D(xPix, yPix);
                        xRel = xy[0];
                        yRel = xy[1];
                        dist = Math.hypot(xRel, yRel);

                        log("(" + xRel + ", " + yRel + ") " + dist);

                        // Save Closest Ring Position to Robot
                        if (0 < yRel && dist < distMin) {
                            distMin = dist;
                            ringPos = new double[] {xRel, yRel};
                        }
                    }
                } else {
                    Imgproc.ellipse(input, ellipse, new Scalar(255, 0, 0), 1);
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

    // Returns ring's position relative to camera
    private double[] map2Dto3D(double xPix, double yPix) {
        double vfov = CAM_VFOV / 2;
        double hfov = CAM_HFOV / 2;
        double num = (Math.cos(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.sin(CAM_PHI);
        double den = (Math.sin(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.cos(CAM_PHI);
        double y = -CAM_HEIGHT * num / den;
        return new double[] {Math.tan(hfov) * xPix * Math.hypot(CAM_HEIGHT, y), y};
    }

    // Returns ring's absolute position using robot's position and ring's position relative to camera
    public double[] getAbsRingPos(double robotX, double robotY, double robotTheta) {
        double ringX = ringPos[0];
        double ringY = ringPos[1] + CAM_FRONT;

        double targetX = robotX + ringX * Math.sin(robotTheta) + ringY * Math.cos(robotTheta);
        double targetY = robotY + ringX * Math.cos(robotTheta) + ringY * Math.sin(robotTheta);
        double targetTheta = Math.atan2(targetY - robotY, targetX - robotX);

        // Ignore ring if it's out of field
        if (targetX < 48 || targetX > 144 || targetY < 80 || targetY > 144) {
            targetX = robotX;
            targetY = robotY;
        }

        return new double[] {targetX, targetY, targetTheta};
    }

    // Returns distance between ring and center of robot
    public double getDist() {
        return Math.hypot(ringPos[0], ringPos[1] + CAM_FRONT);
    }

    // Returns ring's position relative to camera
    public double[] getRelRingPos() {
        return ringPos;
    }

    private void log(String message) {
        Log.w("ring-locator-pipe", message);
    }
}