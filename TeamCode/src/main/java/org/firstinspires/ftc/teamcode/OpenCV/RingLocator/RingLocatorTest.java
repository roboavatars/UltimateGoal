package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Ring Locator Pipeline Test")
public class RingLocatorTest extends LinearOpMode {

    private RingLocator detector;
    private MecanumDrivetrain dt;
    private ArrayList<Ring> rings;

    @Override
    public void runOpMode() {
        dt = new MecanumDrivetrain(this, 111, 63, PI/2);
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            rings = detector.getRings(dt.x, dt.y, dt.theta);

            for (int i = 0; i < rings.size(); i++) {
                if (i == 0) {
                    drawRing(rings.get(i), "green");
                } else if (i == 1) {
                    drawRing(rings.get(i), "yellow");
                } else if (i == 2) {
                    drawRing(rings.get(i), "red");
                }
            }
            drawRobot(dt.x, dt.y, dt.theta, "black");
            drawField();

            addPacket("Rings", rings);
            sendPacket();
        }

        detector.stop();
    }
}