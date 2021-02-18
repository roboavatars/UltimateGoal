package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Ring Locator Pipeline Test")
public class RingLocatorTest extends LinearOpMode {

    private RingLocator detector;
    private ArrayList<Ring> rings;

    @Override
    public void runOpMode() {
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            rings = detector.getRings(87, 33, PI/2);

            for (Ring ring : rings) {
                drawRing(ring);
            }

            addPacket("Rings", rings);
            sendPacket();
        }

        detector.stop();
    }
}