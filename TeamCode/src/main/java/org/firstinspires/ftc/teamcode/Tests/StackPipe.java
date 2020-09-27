package org.firstinspires.ftc.teamcode.Tests;

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
import java.util.List;

@Config
public class StackPipe extends OpenCvPipeline {

    public enum RingCases {None, One, Four}

    public static double min = 85;
    public static double max = 100;
    public static int heightThreshold = 10;
    public static int widthThreshold = 20;

    private double result;
    private Mat yCrCb = new Mat();
    private Mat cb = new Mat();
    private Mat mask = new Mat();

    Mat y = new Mat(); Mat cr = new Mat();
    static final Point REGION_ANCHOR_POINT = new Point(110,85);
    static final int REGION_WIDTH = 100;
    static final int REGION_HEIGHT = 60;
    Point region_pointA = new Point(REGION_ANCHOR_POINT.x, REGION_ANCHOR_POINT.y);
    Point region_pointB = new Point(REGION_ANCHOR_POINT.x + REGION_WIDTH, REGION_ANCHOR_POINT.y + REGION_HEIGHT);
    Mat region1_Cb;

    // clear old imgs
    public StackPipe() {
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

        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCrCb, cb, 2);

        Core.extractChannel(yCrCb, y, 0);
        Core.extractChannel(yCrCb, cr, 1);

        // still not sure how this works
        Core.inRange(cb, new Scalar(min), new Scalar(max), mask);

        // temporary to see avg cb value
        region1_Cb = cb.submat(new Rect(region_pointA, region_pointB));
        int region_cb_mean = (int) Core.mean(region1_Cb).val[0];
        log("Region mean: " + region_cb_mean);
        Imgproc.rectangle(
                input, region_pointA, region_pointB, new Scalar(0, 0, 0), 2);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        saveMatToDisk(input, "input");
        saveMatToDisk(yCrCb, "ycrcb");
        saveMatToDisk(y, "y"); saveMatToDisk(cr, "cr");
        saveMatToDisk(cb, "cb");
        saveMatToDisk(region1_Cb, "region");
        saveMatToDisk(mask, "mask");

        int i = 1;
        for (MatOfPoint contour : contours) {
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);

            if (boundingRect.size.height > heightThreshold && boundingRect.size.width > widthThreshold) {
                Imgproc.rectangle(input, boundingRect.boundingRect(), new Scalar(0, 255, 0), 4);

                saveMatToDisk(input, "result" + i);
                i++;

                result = boundingRect.size.height;
                log("Result: " + result);
            }
        }
        log("Passed w+h threshold: " + i);

        return input;
    }

    public double getResult() {
        return result;
    }

    private void log(String message) {
        Log.w("stack-pipe", message);
    }

}
