package org.firstinspires.ftc.teamcode.OpenCV.StackHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Stack Height Pipeline Test")
public class StackPipelineTest extends LinearOpMode {

    private StackHeightDetector detector;

    @Override
    public void runOpMode() {
        detector = new StackHeightDetector(this);

        waitForStart();
        detector.start();

        while (opModeIsActive()) {
            addPacket("Frame Count", detector.getFrameCount());
            addPacket("FPS", detector.getFPS());
            addPacket("Raw Result", detector.getRawResult()[2]);
            addPacket("Result", detector.getResult());
            addPacket("Mode Result", detector.getModeResult());
            sendPacket();
        }

        detector.stop();
    }
}