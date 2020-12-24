package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Ring Locator Pipeline Test")
public class RingLocatorTest extends LinearOpMode {

    private RingLocator detector;

    @Override
    public void runOpMode() {
        detector = new RingLocator(this);

        waitForStart();
        detector.start();

        while (opModeIsActive()) {
            addPacket("Frame Count", detector.getFrameCount());
            addPacket("FPS", detector.getFPS());
            addPacket("Closest Relative Position", detector.getRelRingPos());
            sendPacket();
        }

        detector.stop();
    }
}