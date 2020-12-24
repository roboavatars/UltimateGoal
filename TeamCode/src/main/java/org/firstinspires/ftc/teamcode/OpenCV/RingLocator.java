package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RingLocator extends BaseDetector {

    private RingLocatorPipeline pipeline;

    public RingLocator(LinearOpMode op) {
        super(op);

        pipeline = new RingLocatorPipeline();
        setPipeline(pipeline);
    }

    public double[] getAbsRingPos(double robotX, double robotY, double robotTheta) {
        return pipeline.getAbsRingPos(robotX, robotY, robotTheta);
    }

    public double[] getRelRingPos() {
        return pipeline.getRelRingPos();
    }
}