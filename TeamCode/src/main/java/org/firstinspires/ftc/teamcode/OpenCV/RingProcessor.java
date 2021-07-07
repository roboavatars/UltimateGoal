package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

@Config
@SuppressLint("SdCardPath")
public class RingProcessor {

    // CV Thresholds
    public static double minY = 0;
    public static double minCr = 0;
    public static double minCb = 80;
    public static double maxY = 255;
    public static double maxCr = 170;
    public static double maxCb = 125;

//    ycrcb -> (0, 0, 80), (255, 170, 125)
//    lab -> a+b: a(0-170) b(160-255)

    public static double amin = 0;
    public static double amax = 170;
    public static double bmin = 160;
    public static double bmax = 255;
    public Scalar labmin = new Scalar(0, amin, bmin);
    public Scalar labmax = new Scalar(255, amax, bmax);

    // Image Processing Mats
    private Mat yCrCb = new Mat();
    private Mat lab = new Mat();

    private Mat processed = new Mat();
    private Mat processedLAB = new Mat();
    private Mat mask = new Mat();
    private Mat maskLAB = new Mat();
    private Mat save;

    private String path = "/sdcard/EasyOpenCV/";

    public RingProcessor(String prefix) {
        path += prefix + '-';
    }

    public Mat[] processFrame(Mat input) {
        // Convert to YCrCb Color Space
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(yCrCb, new Scalar(minY, minCr, minCb), new Scalar(maxY, maxCr, maxCb), processed);

        // lab
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
        Core.inRange(lab, labmin, labmax, processedLAB);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.morphologyEx(processedLAB, processedLAB, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(processedLAB, processedLAB, Imgproc.MORPH_CLOSE, new Mat());

        // Mask Image for Debugging
        mask.release();
        maskLAB.release();
        Core.bitwise_and(input, input, mask, processed);
        Core.bitwise_and(input, input, maskLAB, processedLAB);

        // Save Images for Debug
        saveMatToDisk("input.jpg", input);
        saveMatToDisk("lab.jpg", lab);
        saveMatToDisk("processed.jpg", processed);
        saveMatToDisk("processed3.jpg", processedLAB);
        saveMatToDisk("mask.jpg", mask);
        saveMatToDisk("mask3.jpg", maskLAB);

        return new Mat[] {processed, mask};
    }

    public void saveMatToDisk(String name, Mat mat) {
        save = mat.clone();
        Imgproc.cvtColor(mat, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name, save);
    }
}