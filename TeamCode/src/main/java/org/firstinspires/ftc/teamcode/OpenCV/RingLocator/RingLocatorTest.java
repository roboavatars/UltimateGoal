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
    public static double x = 87;
    public static double y = 63;
    public static double theta = PI/2;

    @Override
    public void runOpMode() {
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            rings = detector.getRings(x, y, theta);

            for (int i = 0; i < rings.size(); i++) {
                if (i == 0) {
                    drawRing(rings.get(i), "green");
                } else if (i == 1) {
                    drawRing(rings.get(i), "red");
                }
            }
            drawRobot(x, y, theta, "black");

            addPacket("Rings", rings);
            sendPacket();
        }

        detector.stop();
    }
}