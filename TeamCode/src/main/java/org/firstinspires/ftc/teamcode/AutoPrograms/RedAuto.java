package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.StackHeightDetector;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeightPipeline.RingCase;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Path;
import org.firstinspires.ftc.teamcode.Splines.Point2D;
import org.firstinspires.ftc.teamcode.Splines.Pose;
import org.firstinspires.ftc.teamcode.Splines.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        /*
        Timeline:
        detect stack
        shoot 3 preloaded rings at powershots
        intake stack rings
        shoot stack rings into high school
        drop off wobble at corresponding zone
        go back to start to pick up second wobble
        drop off second wobble at corresponding zone
        park
        */

        final double PI = Math.PI;

        Robot robot = new Robot(this, 100, 20, PI/2);
        robot.logger.startLogging();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

        boolean startStack = false;
        boolean deliverWobble = false;
        boolean wobbleTwo = false;
        boolean deliverWobble2 = false;
        boolean park = false;

        double startStackTime = 1;
        double deliverWobbleTime = 3;
        double wobbleTwoTime = 5;
        double parkTime = 1;

        waitForStart();

        RingCase ringCase = detector.getResult();
        Point2D[] wobbleDelivery = {
                new Point2D(130, 75), new Point2D(105, 100), new Point2D(125, 120)
        };

        Point2D wobbleCor;
        if (ringCase == RingCase.Four) {
            wobbleCor = wobbleDelivery[2];
            /*deliverWobbleTime =
            wobbleTwoTime = */
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            /*deliverWobbleTime =
            wobbleTwoTime =*/
        } else {
            wobbleCor = wobbleDelivery[0];
            /*deliverWobbleTime =
            wobbleTwoTime =*/
        }

        detector.stop();

        Waypoint[] startStackWaypoints = new Waypoint[] {
                new Waypoint(120.0, 25.0, PI/2, 10.0, 50.0, 0.0, 0.0),
                new Waypoint(111.0, 47.0, 7*PI/12, 10.0, -40.0, 0.0, startStackTime),
        };
        Path startStackPath = new Path(new ArrayList<>(Arrays.asList(startStackWaypoints)));

        Path deliverWobblePath = null;
        Path wobbleTwoPath = null;
        Path deliverWobble2Path = null;
        Path parkPath = null;

        ElapsedTime time = new ElapsedTime();

        while(opModeIsActive()) {

            if (!startStack) {
                double curTime = Math.min(time.seconds(), startStackTime);
                Pose curPose = startStackPath.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > startStackTime + 2) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 50.0, 0.0, 0.0),
                            new Waypoint(wobbleCor.getX(), wobbleCor.getY(), PI/2, 10.0, -40.0, 0.0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    startStack = true; time.reset();
                }
            }

            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > deliverWobbleTime + 2) {
                    Waypoint[] wobbleTwoWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                            new Waypoint(106, 20, PI/2, -10.0, 40.0, 0.0, wobbleTwoTime),
                    };
                    wobbleTwoPath = new Path(new ArrayList<>(Arrays.asList(wobbleTwoWaypoints)));

                    deliverWobble = true; time.reset();
                }
            }

            else if (!wobbleTwo) {
                double curTime = Math.min(time.seconds(), wobbleTwoTime);
                Pose curPose = wobbleTwoPath.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > wobbleTwoTime + 2) {
                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 50.0, 0.0, 0.0),
                            new Waypoint(wobbleCor.getX(), wobbleCor.getY(), PI/2, 10.0, -40.0, 0.0, deliverWobbleTime),
                    };
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    wobbleTwo = true; time.reset();
                }
            }

            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobble2Path.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > deliverWobbleTime + 2) {
                    Waypoint[] parkWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                            new Waypoint(109, 83, PI/2, 10.0, -40.0, 0.0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true; time.reset();
                }
            }

            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > parkTime + 2) {
                    park = true;
                }
            }

            else {
                robot.drivetrain.setControls(0,0,0);
            }

            robot.update();
        }

        robot.update();
        robot.logger.stopLogging();
    }
}