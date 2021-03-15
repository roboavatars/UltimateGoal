package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Auto Ring Detection Test")
public class AutoRingDetectionTest extends LinearOpMode {

    private Robot robot;
    private RingLocator locator;

    private ArrayList<Ring> rings;
    private ArrayList<Waypoint> ringWaypoints;
    private double[] ringPos;
    private Pose finalPose;
    private double ringTime = 0;
    private Path ringPath;

    private double intakePower = 0;
    private boolean start = false;

    private double x;
    private double y;
    private double theta;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 87, 63, PI/2, false);
        robot.intake.blockerDown();
        robot.intake.sticksHome();
        robot.intake.updateSticks();
        locator = new RingLocator(this);
        locator.start();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            x = robot.x;
            y = robot.y;
            theta = robot.theta;

            rings = locator.getRings(x, y, theta);
            addPacket("Rings", rings);
            for (int i = 0; i < rings.size(); i++) {
                if (i == 0) {
                    drawRing(rings.get(i), "green");
                } else if (i == 1) {
                    drawRing(rings.get(i), "red");
                }
            }

            if (start) {
                double curTime = Math.min(timer.seconds(), ringTime);
                Pose curPose = ringPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);
                intakePower = 1;

                if (curTime > ringTime && robot.isAtPose(finalPose.x, finalPose.y, finalPose.theta)) {
                    start = false;
                    intakePower = 0;
                }
            } else {
                if (gamepad1.right_trigger > 0) {
                    intakePower = 1;
                } else if (gamepad1.left_trigger > 0) {
                    intakePower = -1;
                } else if (!start) {
                    intakePower = 0;
                }

                if (gamepad1.left_trigger != 0) {
                    robot.resetOdo(87, 63, PI/2);
                }

//                if (robot.numRings == 0) {
//                    robot.intake.autoSticks(x, y, theta, 6);
//                }

                if (gamepad1.b) {
                    start = true;
                    timer.reset();
                }

                ringWaypoints = new ArrayList<>();
                ringWaypoints.add(new Waypoint(x, y, theta, 50, 60, 0, 0));

                ringTime = 0;
                if (rings.size() >= 1) {
                    ringPos = rings.get(0).driveToRing(x, y);
                    if (ringPos[1] > 135) {
                        ringPos[2] = PI/2;
                    }
                    ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 20, 30, 0, ringTime + 1.5));
                    ringTime += 1.5;
                }
                if (rings.size() == 2) {
                    ringPos = rings.get(1).driveToRing(ringPos[0], ringPos[1]);
                    if (ringPos[1] > 135) {
                        ringPos[2] = PI/2;
                    }
                    ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 20, 30, 0, ringTime + 1.5));
                    ringTime += 1.5;
                }
                ringWaypoints.add(new Waypoint(123, 134, PI, 30, 20, 0, ringTime + 1.5));
                ringTime += 1.5;

                ringPath = new Path(ringWaypoints);
                finalPose = ringPath.getRobotPose(ringTime);
            }

            for (double i = 0; i < ringTime; i += ringTime / 40) {
                try {
                    drawPoint(ringPath.getRobotPose(i).x, ringPath.getRobotPose(i).y, "blue");
                } catch (ArrayIndexOutOfBoundsException ignore) {}
            }

            robot.intake.setPower(intakePower);
            robot.update();
        }

        locator.stop();
    }
}