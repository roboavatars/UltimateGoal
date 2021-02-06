package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;

@Config
public class RingLocator extends BaseDetector {

    private RingLocatorPipeline pipeline;
    public static double minX = Double.MIN_NORMAL; //48;
    public static double minY = Double.MIN_NORMAL; //63;
    public static double maxX = Double.MAX_VALUE; //135;
    public static double maxY = Double.MAX_VALUE; //144;

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
        Log.w("robot-log", "rings0: " + rings);
        for (int i = 0; i < rings.size(); i++) {
            rings.get(i).calcAbsCoords(robotX, robotY, robotTheta);
        }
        Log.w("robot-log", "rings1: " + rings);
        rings = Ring.getRingCoords(rings, minX, minY, maxX, maxY, robotX, robotY);
        Log.w("robot-log", "rings4: " + rings);
        return new ArrayList<>(rings);
    }
}