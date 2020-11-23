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
        move to line
        shoot 3 preloaded rings at powershots
        drop off wobble goal at corresponding zone
        go back to start to pick up second wobble
        intake stack rings
        shoot stack rings into high goal
        drop off second wobble at corresponding zone
        park
        */

        Robot robot = new Robot(this, 90, 9, PI/2);
        robot.t265.startCam();
//        robot.logger.startLogging();

//        StackHeightDetector detector = new StackHeightDetector(this);
//        detector.start();

        boolean startLine = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean intakeWobble2 = false;
        boolean intakeStack = false;
        boolean shootHighGoal = false;
        boolean deliverWobble2 = false;
        boolean park = false;

        double startLineTime = 2.0;
        double shootPowerShotsTime = 4.5;
        double deliverWobbleTime = 2.0;
        double intakeWobble2Time = 3.0;
        double intakeStackTime = 3.0;
        double shootHighGoalTime = 4.5;
        double deliverWobble2Time = 3.0;
        double parkTime = 1;

        waitForStart();

        robot.intake.intakeOff();

//        RingCase ringCase = detector.getResult();
        RingCase ringCase = RingCase.One;
        Point2D[] wobbleDelivery = {
                new Point2D(104, 83), new Point2D(96, 107), new Point2D(104, 131)
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
            intakeStack = true;
            /*deliverWobbleTime =
            wobbleTwoTime =*/
        }

//        detector.stop();

        Waypoint[] startLineWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 20.0, 50.0, 0.0, 0.0),
                new Waypoint(90, 72, PI/2, 10.0, -20.0, 0.0, startLineTime),
        };
        Path startLinePath = new Path(new ArrayList<>(Arrays.asList(startLineWaypoints)));

        Path deliverWobblePath = null;
        Path intakeWobble2Path = null;
        Path intakeStackPath = null;
        Path deliverWobble2Path = null;
        Path parkPath = null;

        ElapsedTime time = new ElapsedTime();

        while(opModeIsActive()) {

            if (!startLine) {
                double curTime = Math.min(time.seconds(), startLineTime);
                Pose curPose = startLinePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                robot.shooter.flywheelOn();

                if (time.seconds() > startLineTime + 1) {
                    startLine = true;
                    time.reset();
                }
            }

            else if (!shootPowerShots) {

                robot.powerShotShoot();

                if (time.seconds() > shootPowerShotsTime + 1) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 40.0, 30.0, 0.0, 0.0),
                            new Waypoint(wobbleCor.getX(), wobbleCor.getY(), 5*PI/6, 50.0, -30.0, 0.0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    shootPowerShots = true;
                    time.reset();
                }
            }

            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                robot.intake.wobbleDown();
                robot.intake.wobbleRelease();

                if (time.seconds() > deliverWobbleTime + 1) {
                    Waypoint[] intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                            new Waypoint(130, 55, PI/2, -20.0, 10.0, 0.0, 1.5),
                            new Waypoint(115, 33, PI/2, -10.0, 20.0, 0.0, intakeWobble2Time),
                    };
                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time);
                Pose curPose = intakeWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                robot.intake.wobbleDown();

                if (time.seconds() > intakeWobble2Time + 1) {
                    Waypoint[] intakeStackWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 20.0, 0.0, 0.0),
                            new Waypoint(112, 41, PI/2, 10.0, -40.0, 0.0, intakeStackTime),
                    };
                    intakeStackPath = new Path(new ArrayList<>(Arrays.asList(intakeStackWaypoints)));

                    robot.intake.wobbleClamp();
                    robot.intake.wobbleUp();

                    intakeWobble2 = true;
                    time.reset();
                }
            }

            else if (!intakeStack) {
                double curTime = Math.min(time.seconds(), intakeStackTime);
                Pose curPose = intakeStackPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                robot.intake.intakeOn();

                if (time.seconds() > intakeStackTime + 1) {
                    intakeStack = true;
                    time.reset();
                }
            }

            else if (!shootHighGoal) {

                robot.intake.intakeOff();

                robot.highGoalShoot();

                if (time.seconds() > shootHighGoalTime + 1) {
                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 30.0, 0.0, 0.0),
                            new Waypoint(wobbleCor.getX(), wobbleCor.getY(), 5*PI/6, 10.0, -30.0, 0.0, deliverWobble2Time),
                    };
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    shootHighGoal = true;
                    time.reset();
                }
            }

            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobble2Time);
                Pose curPose = deliverWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                robot.intake.wobbleDown();
                robot.intake.wobbleRelease();

                if (time.seconds() > deliverWobble2Time + 1) {
                    robot.intake.wobbleUp();

                    Waypoint[] parkWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -30.0, 0.0, 0.0),
                            new Waypoint(98, 80, PI/2, 10.0, 20.0, 0.0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > parkTime + 1) {
                    park = true;
                }
            }

            robot.update();
        }

        robot.update();
        robot.t265.stopCam();
//        robot.logger.stopLogging();
    }
}
