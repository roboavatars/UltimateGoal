package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
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
public class StackHeightPipeline extends OpenCvPipeline {

    public enum RingCase {Zero, One, Four}

    public static double FILTER_MIN = 80;
    public static double FILTER_MAX = 115;
    public static int HEIGHT_THRESH = 3;
    public static int WIDTH_THRESH = 15;
    public static double ONE_MIN = 2.7;
    public static double ONE_MAX = 4.1;
    public static double FOUR_MIN = 1.2;

    private double[] result = new double[3];
    private RingCase ringCase = RingCase.Zero;
    private Mat yCrCb = new Mat();
    private Mat cb = new Mat();
    private Mat processed = new Mat();
    private Mat mask = new Mat();

    // Clear Old Images
    public StackHeightPipeline() {
        @SuppressLint("SdCardPath")
        File dir = new File("/sdcard/EasyOpenCV");
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
        input = new Mat(input, rect);

        // Convert to YCrCb Color Space
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extract Cb
        Core.extractChannel(yCrCb, cb, 2);

        // Filter Colors
        Core.inRange(cb, new Scalar(FILTER_MIN), new Scalar(FILTER_MAX), processed);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

        // Mask Image for Debugging
        input.copyTo(mask, processed);

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Save Images for Debug
        saveMatToDisk(input, "input");
        saveMatToDisk(yCrCb, "ycrcb");
        saveMatToDisk(cb, "cb");
        saveMatToDisk(processed, "processed");
        saveMatToDisk(mask, "mask");

        // Loop Through Contours
        int i = 0;
        for (MatOfPoint contour : contours) {
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);

            // Reject Small Contours
            if (boundingRect.size.height > HEIGHT_THRESH && boundingRect.size.width > WIDTH_THRESH) {

                // Get Detection Size
                Imgproc.rectangle(input, boundingRect.boundingRect(), new Scalar(0, 255, 0), 4);
                i++;

                double width = boundingRect.size.width;
                double height = boundingRect.size.height;
                double wh_ratio = width/height;
                System.out.println(width + " " + height + " " + wh_ratio);

                result = new double[]{width, height, wh_ratio};

                // Checking WH ratio because heights were inconsistent in testing images
                // This works better at a higher camera angle but comparing the height would be better for a lower camera angle.
                if (ONE_MIN <= wh_ratio && wh_ratio <= ONE_MAX) {
                    ringCase = RingCase.One;
                } else if (FOUR_MIN <= wh_ratio && wh_ratio <= ONE_MIN) {
                    ringCase = RingCase.Four;
                }

                log("WH Ratio: " + wh_ratio);
            }
        }

        log("Total: " + contours.size() + " Passed threshold: " + i);

        // No Contours Detected
        if (i == 0) {
            result = new double[]{0,0,0};
            ringCase = RingCase.Zero;
            log("No Contours Detected");
        }

        log("Result: " + Arrays.toString(result));
        log("Case: " + ringCase.name());
        //saveMatToDisk(input, "result" + i);

        return input;
    }

    public double[] getRawResult() {
        return result;
    }

    public RingCase getResult() {
        return ringCase;
    }

    private void log(String message) {
        Log.w("stack-height-pipe", message);
    }
}