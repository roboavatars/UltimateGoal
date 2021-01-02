package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;

@Config @SuppressLint("SdCardPath")
public class RingProcessor {

    // CV Thresholds
    public static double FILTER_MIN = 80;
    public static double FILTER_MAX = 115;

    // Image Processing Mats
    private Mat yCrCb = new Mat();
    private Mat cb = new Mat();
    private Mat processed = new Mat();
    private Mat mask = new Mat();
    private Mat save;

    private final String path = "/sdcard/EasyOpenCV/ringProcessor-";

    public RingProcessor() {
        // Clear Old Images
        File dir = new File(path);
        String[] children = dir.list();
        if (children != null) {
            for (String child : children) {
                new File(dir, child).delete();
            }
        }
    }

    public Mat[] processFrame(Mat input) {
        // Reduce Image Size
        Imgproc.resize(input, input, new Size(320, 180));

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

        // Save Images for Debug
        saveMatToDisk("input.jpg", input);
        saveMatToDisk("ycrcb.jpg", yCrCb);
        saveMatToDisk("cb.jpg", cb);
        saveMatToDisk("processed.jpg", processed);
        saveMatToDisk("mask.jpg", mask);

        return new Mat[] {input, processed, mask};
    }

    public void saveMatToDisk(String name, Mat mat) {
        save = mat.clone(); Imgproc.cvtColor(mat, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name, save);
    }
}