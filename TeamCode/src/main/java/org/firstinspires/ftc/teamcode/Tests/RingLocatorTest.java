package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import java.util.Arrays;

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
            addPacket("Closest Relative Position", Arrays.toString(detector.getRelRingPos()));
            addPacket("FPS", detector.getFPS());
            addPacket("Frame Count", detector.getFrameCount());
            sendPacket();
        }

        detector.stop();
    }
}