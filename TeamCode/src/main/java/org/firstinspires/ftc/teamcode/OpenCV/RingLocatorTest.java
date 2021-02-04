package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.RingLocator;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Ring Locator Pipeline Test")
public class RingLocatorTest extends LinearOpMode {

    private Robot robot;
    private RingLocator detector;
    private double[] ringPos;
    private double intakePower;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 135, 9, Math.PI/2, false);
        robot.intake.blockerDown();
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                intakePower = 1;
            } else if (gamepad1.left_trigger > 0) {
                intakePower = -1;
            } else {
                intakePower = 0;
            }

            if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            } else if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            }

            if (gamepad1.y) {
                robot.shooter.flywheelHG();
            }

            if (gamepad1.left_trigger != 0) {
                robot.resetOdo(135, 9, Math.PI / 2);
            }

            if (robot.numRings == 0) {
                double x = robot.x;
                double y = robot.y;
                double theta = robot.theta;
                double buffer = 6;
                double[] leftPos = new double[] {x - 27 * Math.sin(theta) + 7 * Math.cos(theta), y + 27 * Math.cos(theta) + 7 * Math.sin(theta)};
                double[] rightPos = new double[] {x + 27 * Math.sin(theta) + 7 * Math.cos(theta), y - 27 * Math.cos(theta) + 7 * Math.sin(theta)};
                if (48 + buffer <= leftPos[0] && leftPos[0] <= 144 - buffer && 0 + buffer <= leftPos[1] && leftPos[1] <= 144 - buffer) {
                    robot.intake.stickLeft(Constants.L_OUT_POS);
                } else {
                    robot.intake.stickLeft(Constants.L_HALF_POS);
                }
                if (48 + buffer <= rightPos[0] && rightPos[0] <= 144 - buffer && 0 + buffer <= rightPos[1] && rightPos[1] <= 144 - buffer) {
                    robot.intake.stickRight(Constants.R_OUT_POS);
                } else {
                    robot.intake.stickRight(Constants.R_HALF_POS);
                }
            }

            ringPos = detector.getAbsRingPos(robot.x, robot.y, robot.theta);
            if (!gamepad1.b) {
                robot.drivetrain.setControls(-0.5 * gamepad1.left_stick_y, -0.5 * gamepad1.left_stick_x, -0.5 * gamepad1.right_stick_x);
            } else {
                if (ringPos[0] != -1) {
                    robot.setTargetPoint(ringPos[0], ringPos[1], ringPos[2], 0.30, 0.30, 4);
                    intakePower = 1;
                } else {
                    robot.drivetrain.setControls(0, 0, 0);
                }
            }
            robot.intake.setPower(intakePower);
            robot.update();

            addPacket("Absolute Coords", Arrays.toString(ringPos));
            addPacket("Relative Coords", Arrays.toString(detector.getRelRingPos()));
            addPacket("FPS", detector.getFPS());
            addPacket("Frame Count", detector.getFrameCount());
            sendPacket();
        }

        detector.stop();
    }
}