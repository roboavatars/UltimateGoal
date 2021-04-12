package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawPoint;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRing;

@TeleOp(name = "Auto Ring Detection Test")
public class AutoRingDetectionTest extends LinearOpMode {

    private Robot robot;
    private RingLocator locator;

    private ArrayList<Ring> rings;
    private ArrayList<Waypoint> ringWaypoints;
    private double[] ringPos;
    private double[] ringIntakeTheta = new double[3];
    private double ringTime = 0;
    private Path ringPath;

    private double intakePower = 0;
    private boolean start = false;
    private double startTime, driveTime;

    private double x, y, theta;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 111, 63, PI/2, false);
        locator = new RingLocator(this);
        locator.start();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            x = robot.x;
            y = robot.y;
            theta = robot.theta;

            // Get Ring Positions
            rings = locator.getRings(x, y, theta);
            for (int i = 0; i < rings.size(); i++) {
                if (i == 0) {
                    drawRing(rings.get(i), "green");
                } else if (i == 1) {
                    drawRing(rings.get(i), "yellow");
                } else if (i == 2) {
                    drawRing(rings.get(i), "red");
                }
            }

            if (start) {
                // Follow Path
                double curTime = Math.min(timer.seconds(), ringTime);
                if (rings.size() >= 1 && curTime < 0.75) {
                    robot.setTargetPoint(ringPath.getRobotPose(curTime));
                } else if (rings.size() >= 1 && curTime < 1.5) {
                    robot.setTargetPoint(new Target(ringPath.getRobotPose(curTime)).thetaW0(ringIntakeTheta[0]));
                } else if (rings.size() >= 2 && curTime < 3.0) {
                    robot.setTargetPoint(new Target(ringPath.getRobotPose(curTime)).thetaW0(ringIntakeTheta[1]));
                } else if (rings.size() == 3 && curTime < 4.5) {
                    robot.setTargetPoint(new Target(ringPath.getRobotPose(curTime)).thetaW0(ringIntakeTheta[2]));
                } else {
                    robot.setTargetPoint(new Target(ringPath.getRobotPose(curTime)).thetaW0(PI/2));
                }

                intakePower = 1;
                driveTime = (double) System.currentTimeMillis() / 1000 - startTime;

                // Stop Following if Done
                if (curTime == ringTime && (robot.notMoving() || robot.isAtPose(111, 63, PI/2))) {
                    start = false;
                    intakePower = 0;
                    if (rings.size() > 0) {
                        robot.highGoalShoot(rings.size());
                    }
                }
            } else {
                // Intake Controls
                if (gamepad1.right_trigger > 0) {
                    intakePower = 1;
                } else if (gamepad1.left_trigger > 0) {
                    intakePower = -1;
                } else {
                    intakePower = 0;
                }

                // Drivetrain Controls
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

                // Reset Odo
                if (gamepad1.x) {
                    robot.resetOdo(111, 63, PI/2);
                }

                // Start Path Following
                if (gamepad1.a) {
                    start = true;
                    startTime = (double) System.currentTimeMillis() / 1000;
                    timer.reset();
                }

                // Generate Path
                ringWaypoints = new ArrayList<>();
                ringWaypoints.add(new Waypoint(x, y, theta, 50, 60, 0, 0));

                ringTime = 0;
                if (rings.size() >= 1) {
                    ringPos = rings.get(0).driveToRing(x, y);
                    ringTime += 1.5;
                    if (ringPos[1] > 130) {
                        ringPos[2] = 0;
                        ringIntakeTheta[0] = ringPos[0] - x < 0 ? 3*PI/4 : PI/4;
                    } else {
                        ringIntakeTheta[0] = ringPos[2];
                    }
                    ringWaypoints.add(new Waypoint(ringPos[0], Math.min(130, ringPos[1]), ringIntakeTheta[0], 30, 10, 0, ringTime));
                }

                if (rings.size() >= 2) {
                    ringPos = rings.get(1).driveToRing(ringPos[0], Math.min(130, ringPos[1]));
                    ringTime += 1.5;
                    if (ringPos[1] > 130) {
                        ringPos[2] = 0;
                        ringIntakeTheta[1] = ringPos[0] - rings.get(0).getX() < 0 ? 3*PI/4 : PI/4;
                    } else {
                        ringIntakeTheta[1] = ringPos[2];
                    }
                    ringWaypoints.add(new Waypoint(ringPos[0], Math.min(130, ringPos[1]), ringIntakeTheta[1], 30, 10, 0, ringTime));
                }

                if (rings.size() >= 3) {
                    ringPos = rings.get(2).driveToRing(ringPos[0], Math.min(130, ringPos[1]));
                    ringTime += 1.5;
                    if (ringPos[1] > 130 && rings.get(2).getY() > 130) {
                        ringPos[2] = 0;
                        ringIntakeTheta[2] = ringPos[0] - rings.get(1).getX() < 0 ? 3*PI/4 : PI/4;
                    } else {
                        ringIntakeTheta[2] = ringPos[2];
                    }
                    ringWaypoints.add(new Waypoint(ringPos[0], Math.min(130, ringPos[1]), ringIntakeTheta[2], 30, 10, 0, ringTime));
                }

                ringTime += 1.5;
                ringWaypoints.add(new Waypoint(111, 63, PI/2, -30, -60, 0, ringTime));
                ringPath = new Path(ringWaypoints);
            }

            // Auto Sticks
            robot.intake.autoSticks(x, y, theta, 6);

            // Force Stop
            if (gamepad1.b) {
                start = false;
                intakePower = 0;
                robot.cancelShoot();
                robot.drivetrain.stop();
            }

            // Draw Path
            for (double i = (start ? driveTime : 0); i < ringTime; i += ringTime / 50) {
                try {
                    drawPoint(ringPath.getRobotPose(i).x, ringPath.getRobotPose(i).y, "blue");
                } catch (ArrayIndexOutOfBoundsException ignore) {}
            }

            addPacket("start", start);
            robot.intake.setPower(intakePower);
            robot.update();
        }

        locator.stop();
        robot.stop();
    }
}