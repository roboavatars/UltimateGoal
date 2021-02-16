package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Advanced Ring Locator Pipeline Test")
public class AdvancedRingLocatorTest extends LinearOpMode {

    private Robot robot;
    private RingLocator detector;
    private ArrayList<Ring> rings;
    private double time = 3.0;
    private ArrayList<Waypoint> ringWaypoints = new ArrayList<>(3);
    private Path ringPath;
    private double intakePower;
    private boolean start = false;
    private boolean highGoal = false;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 87, 33, PI/2, false);
        robot.intake.blockerDown();
        detector = new RingLocator(this);
        detector.start();

        rings = detector.getRings(robot.x, robot.y, robot.theta);
        for (Ring ring : rings) {
            drawRing(ring);
        }
        sendPacket();

        waitForStart();

        // camera warmup / buffer frames
        for (int i = 0; i < 100; i++) {
            rings = detector.getRings(robot.x, robot.y, robot.theta);
            for (Ring ring : rings) {
                drawRing(ring);
            }
            sendPacket();
        }
        ElapsedTime timer = new ElapsedTime();

        ringWaypoints.add(new Waypoint(84, 60, PI/2, 60, 100, 0, 0));
        if (rings.size() >= 1) {
            double[] ringPos = rings.get(0).driveToRing(90, 33);
            ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 40, 30, 0, time += 1.0));
        }
        if (rings.size() >= 2) {
            double[] ringPos = rings.get(1).driveToRing(90, 33);
            ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 40, 30, 0, time += 1.0));
        }
        if (rings.size() == 3) {
            double[] ringPos = rings.get(2).driveToRing(90, 33);
            ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 40, 30, 0, time += 1.0));
        }
        ringWaypoints.add(new Waypoint(90, 60, PI/2, -30, -50, 0, time));

        ringPath = new Path(ringWaypoints);

        while (opModeIsActive()) {
            for (Ring ring : rings) {
                drawRing(ring);
            }

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
                robot.resetOdo(135, 9, PI/2);
            }

            if (robot.numRings == 0) {
                robot.intake.autoSticks(robot.x, robot.y, robot.theta, 6);
            }

            if (gamepad1.b || highGoal) {
                robot.drivetrain.setControls(-0.5 * gamepad1.left_stick_y, -0.5 * gamepad1.left_stick_x, -0.5 * gamepad1.right_stick_x);
                timer.reset();
            } else {
                if (!highGoal && time < timer.seconds()) {
//                    robot.highGoalShoot();
                    highGoal = true;
                    intakePower = -1;
                    break;
                } else {
                    if (start) {
                        double curTime = timer.seconds();
                        robot.setTargetPoint(ringPath.getRobotPose(curTime));
                        intakePower = 1;
                    } else {
                        if (robot.isAtPose(84, 60, PI/2, 2, 3, PI/30)) {
                            start = true;
                            timer.reset();
                        } else {
                            robot.setTargetPoint(84, 60, PI/2);
                            intakePower = 0;
                        }
                    }
                }
            }
            robot.intake.setPower(intakePower);
            robot.update();
        }

        detector.stop();
    }
}