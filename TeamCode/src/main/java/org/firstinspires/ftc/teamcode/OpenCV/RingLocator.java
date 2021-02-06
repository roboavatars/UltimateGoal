package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;

@Config
public class RingLocator extends BaseDetector {

    private RingLocatorPipeline pipeline;
    public static double minX = 48;
    public static double minY = 63;
    public static double maxX = 135;
    public static double maxY = 144;

    public RingLocator(LinearOpMode op) {
        super(op);

        pipeline = new RingLocatorPipeline();
        setPipeline(pipeline);
    }

    // Return a sorted list with up to three coordinate-filtered rings
    public ArrayList<Ring> getRings(double robotX, double robotY, double robotTheta) {
        ArrayList<Ring> rings = pipeline.getRings();
        for (int i = 0; i < rings.size(); i++) {
            rings.get(i).calcAbsCoords(robotX, robotY, robotTheta);
        }
        return Ring.getRingCoords(rings, minX, minY, maxX, maxY);
    }
}