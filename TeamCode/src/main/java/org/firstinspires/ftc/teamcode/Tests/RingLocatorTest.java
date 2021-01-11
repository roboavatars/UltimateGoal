package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.RingLocator;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Ring Locator Pipeline Test")
public class RingLocatorTest extends LinearOpMode {

//    private Robot robot;
    private RingLocator detector;
    private double[] ringPos;
    private double intakePower;

    @Override
    public void runOpMode() {
//        robot = new Robot(this, 135, 9, Math.PI/2, false);
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
//            if (gamepad1.right_trigger > 0) {
//                intakePower = 1;
//            } else if (gamepad1.left_trigger > 0) {
//                intakePower = -1;
//            } else {
//                intakePower = 0;
//            }
//
//            if (gamepad1.left_bumper) {
//                robot.highGoalShoot();
//            } else if (gamepad1.right_bumper) {
//                robot.powerShotShoot();
//            }
//
//            if (gamepad1.y) {
//                robot.shooter.flywheelHighGoal();
//            }
//
//            ringPos = detector.getAbsRingPos(robot.x, robot.y, robot.theta);
//            if (!gamepad1.b) {
//                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//            } else {
//                if (ringPos[0] != -1) {
//                    robot.setTargetPoint(ringPos[0], ringPos[1], robot.theta);
//                } else {
//                    robot.drivetrain.setControls(0, 0, 0);
//                }
//                intakePower = 1;
//            }
//            robot.intake.setPower(intakePower);
//            robot.update();

            addPacket("Absolute Coords", Arrays.toString(ringPos));
            addPacket("Relative Coords", Arrays.toString(detector.getRelRingPos()));
            addPacket("FPS", detector.getFPS());
            addPacket("Frame Count", detector.getFrameCount());
            sendPacket();
        }

        detector.stop();
    }
}