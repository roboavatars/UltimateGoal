package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Config
public class StackHeightPipeline extends OpenCvPipeline {

    // Cases
    public enum RingCase {Zero, One, Four}

    // Thresholds
    public static int HEIGHT_THRESH = 3;
    public static int WIDTH_THRESH = 15;
    public static double ONE_MIN = 2.7;
    public static double ONE_MAX = 4.1;
    public static double FOUR_MIN = 1.2;

    // Results
    private double[] result = new double[3];
    private RingCase ringCase = RingCase.Zero;
    private ArrayList<RingCase> results = new ArrayList<>();
    private int cycles = 0;

    // Image Processing Mats
    private RingProcessor processor;
    private Mat processed = new Mat();

    public StackHeightPipeline() {
        // Clear Old Images
        @SuppressLint("SdCardPath") File dir = new File("/sdcard/EasyOpenCV/stackHeight");
        String[] children = dir.list();
        if (children != null) {
            for (String child : children) {
                new File(dir, child).delete();
            }
        }

        processor = new RingProcessor();
        Collections.fill(results, RingCase.Zero);
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process Image
        processed = processor.processFrame(input)[0];

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

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
                log("Loop(" + i + "): " + width + " " + height + " " + wh_ratio);

                result = new double[]{width, height, wh_ratio};

                // Checking WH ratio because heights were inconsistent in testing images
                // This works better at a higher camera angle but comparing the height would be better for a lower camera angle.
                if (ONE_MIN <= wh_ratio && wh_ratio <= ONE_MAX) {
                    ringCase = RingCase.One;
                } else if (FOUR_MIN <= wh_ratio && wh_ratio <= ONE_MIN) {
                    ringCase = RingCase.Four;
                }
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

        results.set(cycles % 5, ringCase);
        cycles++;

        return input;
    }

    public double[] getRawResult() {
        return result;
    }

    public RingCase getResult() {
        return ringCase;
    }

    public RingCase getModeResult() {
        int zero = Collections.frequency(results, RingCase.Zero);
        int one = Collections.frequency(results, RingCase.One);
        int four = Collections.frequency(results, RingCase.Four);
        if (one > zero && one > four) { return RingCase.One; }
        else if (four > zero && four > one) { return RingCase.Four; }
        else { return RingCase.Zero; }
    }

    private void log(String message) {
        Log.w("stack-height-pipe", message);
    }
}