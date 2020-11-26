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

        Robot robot = new Robot(this, 90, 9, PI/2, true);
        robot.logger.startLogging();
        robot.intake.sticksHome();
        robot.shooter.magShoot();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

        boolean startLine = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean intakeWobble2 = false;
        boolean intakeStack = false;
        boolean shootHighGoal = false;
        boolean deliverWobble2 = false;
        boolean park = false;

        double startLineTime = 2.0;
        double shootPowerShotsTime = 4;
        double deliverWobbleTime = 2.0;
        double intakeWobble2Time = 3.0;
        double intakeStackTime = 3.0;
        double shootHighGoalTime = 4;
        double deliverWobble2Time = 3.0;
        double parkTime = 2;

        waitForStart();

        RingCase ringCase = detector.getResult();
        double[][] wobbleDelivery = {{123, 73}, {97, 96}, {123, 121}};
        double[][] wobble2Delivery = {{123, 72}, {97, 93}, {113, 126}};

        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Four) {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
            /*deliverWobbleTime =
            wobbleTwoTime = */
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];
            /*deliverWobbleTime =
            wobbleTwoTime =*/
        } else {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];
            intakeStack = true;
            /*deliverWobbleTime =
            wobbleTwoTime =*/
        }

        detector.stop();

        if (ringCase == RingCase.Four) {
            robot.intake.rStickDown();
        }

        Waypoint[] startLineWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 20.0, 50.0, 0.0, 0.0),
                new Waypoint(90, 68, PI/2, 10.0, -10.0, 0.0, startLineTime),
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

                robot.shooter.setShooterVelocity(-875);

                if (time.seconds() > startLineTime) {
                    robot.intake.sticksHome();
                }

                if (time.seconds() > startLineTime + 1) {

                    robot.powerShotShoot();

                    startLine = true;
                    time.reset();
                }
            }

            else if (!shootPowerShots) {

                if (robot.numRings == 0 || time.seconds() > shootPowerShotsTime + 1) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 40.0, 30.0, 0.0, 0.0),
                            new Waypoint(wobbleCor[0], wobbleCor[1], PI/4, 50.0, -30.0, 0.0, deliverWobbleTime),
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

                if (time.seconds() > deliverWobbleTime) {
                    robot.intake.setIntakePow(0.3);
                    robot.intake.sticksHalf();
                }

                if (time.seconds() > deliverWobbleTime + 1) {

                    robot.intake.sticksHome();
                    robot.intake.wobbleDown();
                    robot.shooter.magHome();

                    Waypoint[] intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                            new Waypoint(130, 60, PI/3, -20.0, 10.0, 0.0, 1.5),
                            new Waypoint(123, 37.5, PI/2, -5.0, -10.0, 0.0, intakeWobble2Time),
                    };
                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time);
                Pose curPose = intakeWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta() + PI);

                if (time.seconds() > intakeWobble2Time + 1) {
                    robot.intake.intakeOn();

                    Waypoint[] intakeStackWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 20.0, 0.0, 0.0),
                            new Waypoint(112, 44, PI/2, 10.0, 0.0, 0.0, 1.5),
                            new Waypoint(108, 61, PI/2, 5.0, -20.0, 0.0, intakeStackTime),
                    };
                    intakeStackPath = new Path(new ArrayList<>(Arrays.asList(intakeStackWaypoints)));

                    if (ringCase != RingCase.Zero) {
                        robot.shooter.setShooterVelocity(-2000);
                    }

                    intakeWobble2 = true;
                    time.reset();
                }
                else if (intakeWobble2Time < time.seconds()) {
                    robot.intake.wobbleClamp();
                }
                else if (intakeWobble2Time - 1 < time.seconds()) {
                    robot.intake.wobbleRelease();
                }
            }

            else if (!intakeStack) {
                double curTime = Math.min(time.seconds(), intakeStackTime);
                Pose curPose = intakeStackPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > intakeStackTime + 1) {
                    robot.highGoalShoot();

                    intakeStack = true;
                    time.reset();
                }
            }

            else if (!shootHighGoal) {
                if (ringCase != RingCase.Zero) {
                    robot.intake.intakeRev();
                } else {
                    robot.numRings = 0;
                }

                if (robot.numRings == 0 || time.seconds() > shootHighGoalTime + 1) {
                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 30.0, 0.0, 0.0),
                            new Waypoint(wobble2Cor[0], wobble2Cor[1], 5*PI/4, 10.0, -30.0, 0.0, deliverWobble2Time),
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

                if (time.seconds() > deliverWobble2Time + 1.5) {
                    robot.intake.wobbleUp();

                    Waypoint[] parkWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -20.0, 0.0, 0.0),
                            new Waypoint(98, 80, 3*PI/2, 10.0, 20.0, 0.0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                } else if (time.seconds() > deliverWobble2Time) {
                    robot.intake.wobbleRelease();
                } else if (time.seconds() > deliverWobble2Time - 1) {
//                    robot.intake.wobbleDown();
                }
            }

            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta() + PI, 0.17, 0.17, 1.9);

                if (time.seconds() > 1) {
                    robot.intake.wobbleUp();
                }

                if (robot.isAtPose(curPose.getX(), curPose.getY(), PI/2)) {
                    robot.intake.sticksOut();
                    robot.intake.intakeOff();
                    robot.intake.wobbleUp();
                    park = true;
                }
            }

            else {
                robot.drivetrain.setControls(0, 0, 0);
            }

            robot.update();
        }

        robot.intake.wobbleUp();
        robot.shooter.feedHome();

        robot.update();
        robot.stop();
    }
}
