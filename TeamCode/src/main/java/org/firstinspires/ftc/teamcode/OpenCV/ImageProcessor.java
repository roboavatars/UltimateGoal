package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;

@Config
public class ImageProcessor {

    // CV Thresholds
    public static double FILTER_MIN = 80;
    public static double FILTER_MAX = 115;

    private final String path = "/sdcard/EasyOpenCV/imageProcessor";

    // Image Processing Mats
    private Mat yCrCb = new Mat();
    private Mat cb = new Mat();
    private Mat processed = new Mat();
    private Mat mask = new Mat();

    public ImageProcessor() {
        // Clear Old Images
        @SuppressLint("SdCardPath") File dir = new File("/sdcard/EasyOpenCV/imageProcessor");
        String[] children = dir.list();
        if (children != null) {
            for (String child : children) {
                new File(dir, child).delete();
            }
        }
    }

    public Mat[] processFrame(Mat input) {
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

        // Save Images for Debug
        Imgcodecs.imwrite(path + "input.jpg", input);
        Imgcodecs.imwrite(path + "ycrcb.jpg", yCrCb);
        Imgcodecs.imwrite(path + "cb.jpg", cb);
        Imgcodecs.imwrite(path + "processed.jpg", processed);
        Imgcodecs.imwrite(path + "mask.jpg", mask);

        return new Mat[] {processed, mask};
    }
}