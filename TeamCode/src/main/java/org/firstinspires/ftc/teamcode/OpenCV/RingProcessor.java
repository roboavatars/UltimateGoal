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
    public static double cbmin1 = 80;
    public static double cbmax1 = 110;
    public static double cbmin2 = 70;
    public static double cbmax2 = 95;

//    ycrcb ->cb: 70-95
//    lab -> a+b: a(0-170) b(160-255)

    public static double amin = 0;
    public static double amax = 170;
    public static double bmin = 160;
    public static double bmax = 255;
    public Scalar labmin = new Scalar(0, amin, bmin);
    public Scalar labmax = new Scalar(255, amax, bmax);

    // Image Processing Mats
    private Mat yCrCb = new Mat();
    private Mat cb = new Mat();
    private Mat lab = new Mat();

    private Mat processed = new Mat();
    private Mat processed2 = new Mat();
    private Mat processed3 = new Mat();
    private Mat mask = new Mat();
    private Mat mask2 = new Mat();
    private Mat mask3 = new Mat();
    private Mat save;

    private String path = "/sdcard/EasyOpenCV/";

    public RingProcessor(String prefix) {
        path += prefix + '-';
    }

    public Mat[] processFrame(Mat input) {
        // Convert to YCrCb Color Space
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extract Cb
        Core.extractChannel(yCrCb, cb, 2);

        // Filter Colors
        Core.inRange(cb, new Scalar(cbmin1), new Scalar(cbmax1), processed);
        Core.inRange(cb, new Scalar(cbmin2), new Scalar(cbmax2), processed2);

        // lab
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
        Core.inRange(lab, labmin, labmax, processed3);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.morphologyEx(processed2, processed2, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.morphologyEx(processed3, processed3, Imgproc.MORPH_CLOSE, new Mat());

        // Mask Image for Debugging
        mask.release();
        mask2.release();
        mask3.release();
        Core.bitwise_and(input, input, mask, processed);
        Core.bitwise_and(input, input, mask2, processed2);
        Core.bitwise_and(input, input, mask3, processed3);

        // Save Images for Debug
        saveMatToDisk("input.jpg", input);
        saveMatToDisk("cb.jpg", cb);
        saveMatToDisk("lab.jpg", lab);
        saveMatToDisk("processed.jpg", processed);
        saveMatToDisk("processed2.jpg", processed2);
        saveMatToDisk("processed3.jpg", processed3);
        saveMatToDisk("mask.jpg", mask);
        saveMatToDisk("mask2.jpg", mask2);
        saveMatToDisk("mask3.jpg", mask3);

        return new Mat[] {processed, mask};
    }

    public void saveMatToDisk(String name, Mat mat) {
        save = mat.clone();
        Imgproc.cvtColor(mat, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name, save);
    }
}