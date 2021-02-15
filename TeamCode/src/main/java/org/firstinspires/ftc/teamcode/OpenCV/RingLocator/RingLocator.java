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
    public static double minY = 72;
    public static double maxX = 132;
    public static double maxY = 144;

    public RingLocator(LinearOpMode op) {
        super(op);

        pipeline = new RingLocatorPipeline();
        setPipeline(pipeline);
    }

    public ArrayList<Ring> getRawRings() {
        return pipeline.getRings();
    }

    // Return a sorted list with up to three coordinate-filtered rings
    public ArrayList<Ring> getRings(double robotX, double robotY, double robotTheta) {
        ArrayList<Ring> rings = pipeline.getRings();
        pipeline.log("rings0: " + rings + "\n");
        for (int i = 0; i < rings.size(); i++) {
            try {
                rings.get(i).calcAbsCoords(robotX, robotY, robotTheta);
            } catch (Exception ignore) {}
        }
        pipeline.log("rings1: " + rings + "\n");
        rings = Ring.getRingCoords(rings, minX, minY, maxX, maxY, robotX, robotY);
        pipeline.log("rings4: " + rings + "\n");
        return new ArrayList<>(rings);
    }
}