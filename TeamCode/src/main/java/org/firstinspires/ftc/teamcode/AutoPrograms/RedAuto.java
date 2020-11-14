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
import static java.lang.Math.PI;

@Autonomous
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        /*
        Timeline:
        detect stack
        shoot 3 preloaded rings at powershots
        intake stack rings
        shoot stack rings into high goal
        drop off wobble goal at corresponding zone
        go back to start to pick up second wobble
        drop off second wobble at corresponding zone
        park
        */

        Robot robot = new Robot(this, 115, 9, PI/2);
        robot.logger.startLogging();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

        boolean startStack = false;
        boolean deliverWobble = false;
        boolean wobbleTwo = false;
        boolean deliverWobble2 = false;
        boolean park = false;

        double startStackTime = 2;
        double deliverWobbleTime = 3;
        double wobbleTwoTime = 4;
        double parkTime = 1;

        waitForStart();

        RingCase ringCase = detector.getResult();
        Point2D[] wobbleDelivery = {
                new Point2D(118, 83), new Point2D(110, 107), new Point2D(122, 131)
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
                new Waypoint(115, 9, PI/2, 5, 5, 0.0, 0.0),
                new Waypoint(109, 20, PI/2, 20, 30, 0.0, 0.75),
                new Waypoint(110, 45, PI/2, 20.0, -40.0, 0.0, startStackTime),
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
                double[] angles;

                if (time.seconds() <= 0.3) {
                    angles = robot.shoot(0);
                    robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), angles[0]);
                    robot.shooter.setFlapAngle(angles[1]);
                } else if (time.seconds() <= 0.6) {
                    angles = robot.shoot(1);
                    robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), angles[0]);
                    robot.shooter.setFlapAngle(angles[1]);
                } else if (time.seconds() <= 0.9) {
                    angles = robot.shoot(2);
                    robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), angles[0]);
                    robot.shooter.setFlapAngle(angles[1]);
                } else {
                    robot.intake.intakeOn();
                    robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());
                }

                if (time.seconds() > startStackTime + 1) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 50.0, 0.0, 0.0),
                            new Waypoint(wobbleCor.getX(), wobbleCor.getY(), PI/2, 10.0, -40.0, 0.0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    robot.intake.intakeOff();

                    startStack = true;
                    time.reset();
                }
            }

            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > deliverWobbleTime + 1) {
                    Waypoint[] wobbleTwoWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                            new Waypoint(111, 26, PI/2, -10.0, 40.0, 0.0, wobbleTwoTime),
                    };
                    wobbleTwoPath = new Path(new ArrayList<>(Arrays.asList(wobbleTwoWaypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            else if (!wobbleTwo) {
                double curTime = Math.min(time.seconds(), wobbleTwoTime);
                Pose curPose = wobbleTwoPath.getRobotPose(curTime);
                double[] angles;

                if (time.seconds() <= 2) {
                    angles = robot.shoot(3);
                    robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), angles[0]);
                    robot.shooter.setFlapAngle(angles[1]);
                } else {
                    robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());
                }

                if (time.seconds() > wobbleTwoTime + 1) {
                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 50.0, 0.0, 0.0),
                            new Waypoint(wobbleCor.getX(), wobbleCor.getY(), PI/2, 10.0, -40.0, 0.0, deliverWobbleTime),
                    };
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    wobbleTwo = true;
                    time.reset();
                }
            }

            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobble2Path.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > deliverWobbleTime + 1) {
                    Waypoint[] parkWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                            new Waypoint(109, 83, PI/2, 10.0, 40.0, 0.0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.drivetrain.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > parkTime + 1) {
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
