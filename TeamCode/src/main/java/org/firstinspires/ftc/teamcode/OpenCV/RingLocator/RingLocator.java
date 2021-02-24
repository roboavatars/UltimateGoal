package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Ring;

import java.util.ArrayList;

@Config
public class RingLocator extends BaseDetector {

    private RingLocatorPipeline pipeline;
    public static double minX = 60;
    public static double minY = 94;
    public static double maxX = 132;
    public static double maxY = 146;

    public RingLocator(LinearOpMode op) {
        super(op);

        pipeline = new RingLocatorPipeline();
        setPipeline(pipeline);
    }

    public ArrayList<Ring> getRawRings() {
        return pipeline.getRings();
    }

    // Return rings
    public ArrayList<Ring> getRings() {
        return new ArrayList<>(pipeline.getRings());
    }

    // Return a sorted list with up to two coordinate-filtered rings
    public ArrayList<Ring> getRings(double robotX, double robotY, double robotTheta) {
        return new ArrayList<>(pipeline.getRings(robotX, robotY, robotTheta));
    }
}