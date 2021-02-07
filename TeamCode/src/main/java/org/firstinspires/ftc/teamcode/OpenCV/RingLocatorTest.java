package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Ring Locator Pipeline Test")
public class RingLocatorTest extends LinearOpMode {

    private Robot robot;
    private RingLocator detector;
    private ArrayList<Ring> rings;
    private double time = 3.0;
    private ArrayList<Waypoint> ringWaypoints = new ArrayList<>(3);
    private Path ringPath;
    private double intakePower;
    private boolean highGoal = false;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 89, 33, PI/2, false);
        robot.intake.blockerDown();
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        sleep(1000);
        ElapsedTime timer = new ElapsedTime();
        rings = detector.getRings(robot.x, robot.y, robot.theta);
        
        ringWaypoints.add(new Waypoint(robot.x, robot.y, robot.theta, 50, 60, 0, 0));
        if (rings.size() >= 1) {
            time += 3.0;
            double[] ringPos = rings.get(0).driveToRing(90, 33);
            ringWaypoints.add(new Waypoint(ringPos[0] - 4, ringPos[1] - 4, PI/4, 30, 40, 0, 1.0));
            ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] + 4, PI/4, 30, 40, 0, 2.0));
            if (rings.size() >= 2) {
                double ringY = rings.get(1).driveToRing(90, 33)[1];
                ringWaypoints.add(new Waypoint(ringPos[0], ringY - 15, PI/2, -30, -10, 0, 3.0));
            } else {
                ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] - 10, PI/2, -30, -10, 0, 3.0));
            }
        }
        if (rings.size() >= 2) {
            time += 3.0;
            double[] ringPos = rings.get(1).driveToRing(90, 33);
            ringWaypoints.add(new Waypoint(ringPos[0] - 4, ringPos[1] - 4, PI/4, 30, 40, 0, 4.0));
            ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] + 4, PI/4, 30, 40, 0, 5.0));
            if (rings.size() == 3) {
                double ringY = rings.get(2).driveToRing(90, 33)[1];
                ringWaypoints.add(new Waypoint(ringPos[0], ringY - 15, PI/2, -30, -10, 0, 6.0));
            } else {
                ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] - 10, PI/2, -30, -10, 0, 6.0));
            }
        }
        if (rings.size() == 3) {
            time += 3.0;
            double[] ringPos = rings.get(2).driveToRing(90, 33);
            ringWaypoints.add(new Waypoint(ringPos[0] - 4, ringPos[1] - 4, PI/4, 30, 40, 0, 7.0));
            ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] + 4, PI/4, 30, 40, 0, 8.0));
            ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1] - 10, PI/2, -30, -10, 0, 9.0));
        }
        ringWaypoints.add(new Waypoint(90, 60, PI/2, 30, 40, 0, time));

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
                double x = robot.x;
                double y = robot.y;
                double theta = robot.theta;
                double buffer = 8;
                double[] leftPos = new double[] {x - 33 * Math.sin(theta) + 7 * Math.cos(theta), y + 33 * Math.cos(theta) + 7 * Math.sin(theta)};
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

            if (!gamepad1.b || highGoal) {
                robot.drivetrain.setControls(-0.5 * gamepad1.left_stick_y, -0.5 * gamepad1.left_stick_x, -0.5 * gamepad1.right_stick_x);
                timer.reset();
            } else {
                if (!highGoal && time < timer.seconds()) {
                    robot.highGoalShoot();
                    highGoal = true;
                } else {
                    if ((0.5 < time && time < 2.0) || (3.5 < time && time < 5.0) || (6.5 < time && time < 8.0)) {
                        robot.setTargetPoint(ringPath.getRobotPose(timer.seconds()), PI/4, 0);
                    } else {
                        robot.setTargetPoint(ringPath.getRobotPose(timer.seconds()), PI/2, 0);
                    }
                }
                intakePower = 1;
            }
            robot.intake.setPower(intakePower);
            robot.update();
            sendPacket();
        }

        detector.stop();
    }
}