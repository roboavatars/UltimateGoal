package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.OpenCV.RingProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class RingLocatorPipeline extends OpenCvPipeline {

    // CV Thresholds
    public static double HEIGHT_MIN = 15;
    public static double HEIGHT_MAX = 130;
    public static double WIDTH_MIN = 3;
    public static double WIDTH_MAX = 75;
    public static double ANGLE_MIN = 70;
    public static double ANGLE_MAX = 110;

    // Camera Constants
    public static double CAM_HEIGHT = 8;
    public static double CAM_FRONT = 7; // y distance between camera and robot center
    public static double CAM_PHI = Math.toRadians(12.8);
    public static double CAM_VFOV = Math.toRadians(45);
    public static double CAM_HFOV = Math.toRadians(58);

    // Image Processing Mats
    private RingProcessor processor;
    private Mat[] processorOut = new Mat[3];
    private Mat processed = new Mat();

    // Ellipse Variables
    private double width;
    private double height;
    private double angle;
    private double xPix;
    private double yPix;

    // Ring Position Variables
    private ArrayList<Ring> rings = new ArrayList<>();
    private ArrayList<Ring> prevRings = new ArrayList<>();

    public RingLocatorPipeline() {
        processor = new RingProcessor("locator");
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process Image
        processed = processor.processFrame(input)[0];

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop Through Contours
        for (MatOfPoint contour : contours) {
            Point[] contourArray = contour.toArray();

            // Bound Ellipse if Contour is Large Enough
            if (contourArray.length >= 5) {
                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                RotatedRect ellipse = Imgproc.fitEllipse(areaPoints);

                // Save Ellipse Data
                width = ellipse.size.width;
                height = ellipse.size.height;
                angle = ellipse.angle;
                xPix = ellipse.center.x / 160 - 1; // x ∈ [-1, 1]
                yPix = 1 - ellipse.center.y / 180; // y ∈ (0, 1)

//                log("x: " + xPix);
//                log("y: " + yPix);
//                log("width: " + width);
//                log("height: " + height);

                // Analyze Valid Contours
                if (HEIGHT_MIN < height && height < HEIGHT_MAX && WIDTH_MIN < width && width < WIDTH_MAX && ANGLE_MIN < angle && angle < ANGLE_MAX) {
                    Imgproc.ellipse(input, ellipse, new Scalar(0, 255, 0), 1);

                    // Calculate Center of Ring if Y is in Valid Domain
                    if (0 < yPix && yPix < 0.7296) {
                        double[] xy = map2Dto3D(xPix, yPix);
                        Ring curRing = new Ring(xy[0], xy[1]);

                        log("width, height, angle: " + width + ", " + height + ", " + angle);
                        log("(" + curRing.getRelX() + ", " + curRing.getRelY() + ") " + curRing.getRelDist());

                        // Save Ring Position
                        if (curRing.getRelY() > 0) {
                            rings.add(curRing);
                        }
                    }
                } else {
                    Imgproc.ellipse(input, ellipse, new Scalar(255, 0, 0), 1);
                }
            }
        }

        // Return (0, 0) If No Rings Detected
        if (rings.size() == 0) {
            rings.add(new Ring(0, 0));
            log("No Rings Detected");
        }

        prevRings.clear();
        for (Ring ring : rings) {
            prevRings.add(ring.clone());
        }
        rings.clear();

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

    // Return rings
    public ArrayList<Ring> getRings() {
        return new ArrayList<>(prevRings);
    }

    // Return a sorted list with up to two coordinate-filtered rings
    public ArrayList<Ring> getRings(double robotX, double robotY, double robotTheta) {
        ArrayList<Ring> rings = getRings();
        int i = 0;
        while (i < rings.size()) {
            try {
                rings.get(i).calcAbsCoords(robotX, robotY, robotTheta);
                i++;
            } catch (NullPointerException e) {
                rings.remove(i);
            }
        }

        rings = Ring.getRingCoords(rings, robotX, robotY);

        return new ArrayList<>(rings);
    }

    public void log(String message) {
        Log.w("ring-locator-pipe", message);
    }
}