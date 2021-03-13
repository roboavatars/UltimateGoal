package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.RingLocator.RingLocatorPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightPipeline;

public class Vision extends BaseDetector {

    private StackHeightPipeline stackHeightPipeline;
    private RingLocatorPipeline ringLocatorPipeline;

    public enum Pipeline {StackHeight, RingLocator}

    public Vision(LinearOpMode op) {
        super(op);

        stackHeightPipeline = new StackHeightPipeline();
        ringLocatorPipeline = new RingLocatorPipeline();
    }

    public Vision(LinearOpMode op, Pipeline pipeline) {
        super(op);

        stackHeightPipeline = new StackHeightPipeline();
        ringLocatorPipeline = new RingLocatorPipeline();

        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == Pipeline.StackHeight) {
            setPipeline(stackHeightPipeline);
        } else if (pipeline == Pipeline.RingLocator) {
            setPipeline(ringLocatorPipeline);
        }
    }

    public StackHeightPipeline getStackPipe() {
        return stackHeightPipeline;
    }

    public RingLocatorPipeline getRingPipe() {
        return ringLocatorPipeline;
    }
}
