package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
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
import java.util.List;

@Config
public class StackHeightPipeline extends OpenCvPipeline {

    public enum RingCase {None, One, Four}

    public static double FILTER_MIN = 80;
    public static double FILTER_MAX = 110;
    public static int HEIGHT_THRESH = 15;
    public static int WIDTH_THRESH = 15;

    private double[] result = new double[3];
    private RingCase ringCase = RingCase.None;
    private Mat yCrCb = new Mat();
    private Mat cb = new Mat();
    private Mat mask = new Mat();

    Mat y = new Mat();
    Mat cr = new Mat();
    /*static final Point REGION_ANCHOR_POINT = new Point(110,85);
    static final int REGION_WIDTH = 100;
    static final int REGION_HEIGHT = 60;
    Point pointA = new Point(REGION_ANCHOR_POINT.x, REGION_ANCHOR_POINT.y);
    Point pointB = new Point(REGION_ANCHOR_POINT.x + REGION_WIDTH, REGION_ANCHOR_POINT.y + REGION_HEIGHT);
    Mat region_Cb;*/

    // Clear Old Images
    public StackHeightPipeline() {
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

        // crop 10% of each edges
        input = input.submat(input.height()/10, input.height()-(input.height()/10),
                input.width()/10, input.width()-(input.width()/10));

        // convert to ycrcb color space
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);

        // extract cb channel
        Core.extractChannel(yCrCb, cb, 2);
        Core.extractChannel(yCrCb, y, 0); Core.extractChannel(yCrCb, cr, 1);

        // filter out all colors except ring color
        Core.inRange(cb, new Scalar(FILTER_MIN), new Scalar(FILTER_MAX), mask);

        // remove noise in image
        Imgproc.morphologyEx(cb, cb, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(cb, cb, Imgproc.MORPH_CLOSE, new Mat());

        // Temporary to See Average Cb Value
        /*region_Cb = cb.submat(new Rect(pointA, pointB));
        int region_cb_mean = (int) Core.mean(region_Cb).val[0];
        log("Region Mean: " + region_cb_mean);
        Imgproc.rectangle(input, pointA, pointB, new Scalar(0, 0, 0), 2);*/

        // find rings
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // save files for debug
        saveMatToDisk(input, "input");
        saveMatToDisk(yCrCb, "ycrcb");
        saveMatToDisk(y, "y");
        saveMatToDisk(cr, "cr");
        saveMatToDisk(cb, "cb");
        saveMatToDisk(mask, "mask");

        // go through detections
        int i = 0;
        for (MatOfPoint contour : contours) {
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);

            // throw out detections that are too small
            if (boundingRect.size.height > HEIGHT_THRESH && boundingRect.size.width > WIDTH_THRESH) {

                // to get width and height of detection
                Imgproc.rectangle(input, boundingRect.boundingRect(), new Scalar(0, 255, 0), 4);
                i++;

                double width = boundingRect.size.width;
                double height = boundingRect.size.height;
                double wh_ratio = width/height;
                System.out.println(width + " " + height + " " + wh_ratio);

                result = new double[]{width, height, wh_ratio};

                // one ring generally rectangular, four rings generally more square like
                // did this as heights were inconsistent in the sample images
                // works as sample imgs were taken at a high camera angle
                // this won't work for lower angles, height would be better then
                if (wh_ratio < 0.9 || wh_ratio > 1.1) {
                    ringCase = RingCase.One;
                } else if (wh_ratio >= 0.9 || wh_ratio <= 1.1) {
                    ringCase = RingCase.Four;
                }
            }
        }

        log("Total: " + contours.size() + " Passed threshold: " + i);

        // no detections
        if (i == 0) {
            result = new double[]{0,0,0};
            ringCase = RingCase.None;
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