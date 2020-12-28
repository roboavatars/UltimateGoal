package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.StackHeightDetector;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeightPipeline.RingCase;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Spline;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import static java.lang.Math.PI;

@Autonomous(name = "1 RedAuto")
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
        robot.intake.sticksHomeAuto();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

        // Segments
        boolean startLine = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean intakeWobble2 = false;
        boolean intakeStack = false;
        boolean shootHighGoal = false;
        boolean deliverWobble2 = false;
        boolean park = false;

        // Segment Times
        double startLineTime = 1.75;
        double shootPowerShotsTime = 4.0;
        double deliverWobbleTime = 1.5;
        double intakeWobble2Time = 4.0;
        double intakeStackTime = 4.0;
        double shootHighGoalTime = 3.0;
        double deliverWobble2Time = 2.5;
        double parkTime = 2.0;

        waitForStart();

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getModeResult(); //getResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{121, 82}, {96, 103}, {124, 124}};
        double[][] wobble2Delivery = {{117, 74}, {96, 92}, {120, 118}};
        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];
            intakeWobble2Time -= 1;
            intakeStack = true;
            deliverWobble2Time -= 0.5;
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];
            intakeStackTime -= 1;
        } else {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
            deliverWobbleTime += 1;
            deliverWobble2Time += 0.5;
        }

        detector.stop();

        Waypoint[] startLineWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 40, 50, 0, 0),
                new Waypoint(90, 68, PI/2, 10, -30, 0, startLineTime),
        };
        Path startLinePath = new Path(new ArrayList<>(Arrays.asList(startLineWaypoints)));

        Path deliverWobblePath = null;
        Spline deliverWobbleThetaSpline = null;
        Path intakeWobble2Path = null;
        Spline intakeWobble2ThetaSpline = null;
        Path intakeStackPath = null;
        Spline intakeStackThetaSpline = null;
        Path deliverWobble2Path = null;
        Spline deliverWobble2ThetaSpline = null;
        Path parkPath = null;
        Spline parkThetaSpline = null;

        ElapsedTime time = new ElapsedTime();

        while(opModeIsActive()) {

            // Go to shooting line to shoot powershots
            if (!startLine) {
                double curTime = Math.min(time.seconds(), startLineTime);
                Pose curPose = startLinePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                robot.shooter.flywheelPowershot();

                if (time.seconds() > startLineTime - 1) {
                    robot.wobbleArm.setArmPosition(-300);
                }

                if (time.seconds() > startLineTime + 0.5) {

                    robot.powerShotShoot();

                    startLine = true;
                    time.reset();
                }
            }

            // Time block to shoot powershots
            else if (!shootPowerShots) {

                if (!robot.preShoot && !robot.shoot || time.seconds() > shootPowerShotsTime) {

                    Waypoint[] deliverWobbleWaypoints;
                    if (ringCase == RingCase.Zero) {
                        deliverWobbleWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 20, 30, 0, 0),
                                new Waypoint(wobbleCor[0], wobbleCor[1], 13*PI/12, 10, -20, 0, deliverWobbleTime),
                        };
                    } else {
                        deliverWobbleWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 30, 30, 0, 0),
                                new Waypoint(wobbleCor[0], wobbleCor[1], 13*PI/12, 20, -20, 0, deliverWobbleTime),
                        };
                    }
                    deliverWobbleThetaSpline = new Spline(robot.theta, 0.3, 0, 13*PI/12, 0, 0, deliverWobbleTime);
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    shootPowerShots = true;
                    time.reset();
                }
            }

            // Deliver first wobble goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), deliverWobbleThetaSpline.position(curTime));

                if (time.seconds() > deliverWobbleTime + 1) {
                    robot.wobbleArm.unClampWobble();
                } else if (time.seconds() > deliverWobbleTime - 0.5) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > deliverWobbleTime + 1.5) {

                    robot.wobbleArm.clampWobble();
                    robot.wobbleArm.setArmPosition(-200);

                    Waypoint[] intakeWobble2Waypoints;
                    if (ringCase == RingCase.Zero) {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -10, -50, 0, 0),
                                new Waypoint(wobbleCor[0]-4, wobbleCor[1]-5, robot.theta, -10, -50, 0, 0.25),
                                new Waypoint(127, 63, PI/2, -20, -5, 0, 1),
                                new Waypoint(124.5, 37, 5*PI/12, 0, 30, 0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(robot.theta, 0.7, 0, 5*PI/12, 0, 0, intakeWobble2Time);
                    } else if (ringCase == RingCase.One) {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -10, -50, 0, 0),
                                new Waypoint(wobbleCor[0]-4, wobbleCor[1]-5, robot.theta, -10, -50, 0, 0.25),
                                new Waypoint(127, 66, PI/2, -30, -5, 0, 1.5),
                                new Waypoint(124.5, 37, 5*PI/12, 0, 30, 0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(robot.theta, 0.5, 0, 5*PI/12, 0, 0, intakeWobble2Time);
                    } else {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -30, -50, 0, 0),
                                new Waypoint(wobbleCor[0]-4, wobbleCor[1]-5, robot.theta, -10, -50, 0, 0.25),
                                new Waypoint(127, 66, PI/2, -40, -5, 0, 2),
                                new Waypoint(124.5, 37, 5*PI/12, 0, 30, 0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(robot.theta, 0.5, 0, 5*PI/12, 0, 0, intakeWobble2Time);
                    }
                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to pick up second wobble goal
            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time);
                Pose curPose = intakeWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), intakeWobble2ThetaSpline.position(curTime));

                if (time.seconds() > intakeWobble2Time + 0.75) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (time.seconds() > intakeWobble2Time) {
                    robot.wobbleArm.clampWobble();
                } else if (time.seconds() > intakeWobble2Time - 1.5) {
                    robot.wobbleArm.unClampWobble();
                } else if (time.seconds() > intakeWobble2Time - 2) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > intakeWobble2Time + 1.5) {
                    if (ringCase != RingCase.Zero) {
                        robot.intake.intakeOn();
                        robot.shooter.flywheelHighGoal();
                    }

                    Waypoint[] intakeStackWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -10, -30, 0, 0),
                            new Waypoint(robot.x - 4, robot.y - 4, 2*PI/3, 20, 20, 0, 0.25),
                            new Waypoint(118, 38, 2*PI/3, 10, 10, 0, 1.5),
                            new Waypoint(106, 52, 2*PI/3, 10, 10, 0, intakeStackTime),
                    };
                    intakeStackThetaSpline = new Spline(robot.theta, 4, 0, 2*PI/3, 0, 0, intakeStackTime);
                    intakeStackPath = new Path(new ArrayList<>(Arrays.asList(intakeStackWaypoints)));

                    intakeWobble2 = true;
                    time.reset();
                }
            }

            // Intake start stack
            else if (!intakeStack) {
                double curTime = Math.min(time.seconds(), intakeStackTime);
                Pose curPose = intakeStackPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), intakeStackThetaSpline.position(curTime));

//                if (time.seconds() > intakeStackTime - 0.5) {
//                    robot.vibrateMag = true;
//                    robot.vibrateTime = System.currentTimeMillis();
//                }

                if (time.seconds() > intakeStackTime) {

                    robot.highGoalShoot();
                    robot.intake.intakeRev();

                    intakeStack = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal) {

                if (ringCase == RingCase.Zero) {
                    robot.numRings = 0;
                }

                if (!robot.preShoot && !robot.shoot || time.seconds() > shootHighGoalTime) {

                    robot.intake.intakeOff();

                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 10, 30, 0, 0),
                            new Waypoint(wobble2Cor[0], wobble2Cor[1], 5*PI/4, 15, -20, 0, deliverWobble2Time),
                    };
                    deliverWobble2ThetaSpline = new Spline(robot.theta, 0.3, 0, 5*PI/4, 0, 0, deliverWobble2Time);
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    shootHighGoal = true;
                    time.reset();
                }
            }

            // Deliver second wobble goal
            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobble2Time);
                Pose curPose = deliverWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), deliverWobble2ThetaSpline.position(curTime));

                if (time.seconds() > deliverWobble2Time + 0.5) {
                    robot.wobbleArm.unClampWobble();
                } else if (time.seconds() > deliverWobble2Time - 0.5) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > deliverWobble2Time + 1) {

                    robot.wobbleArm.armUp();

                    Waypoint[] parkWaypoints;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta + PI, -10, -20, 0, 0),
                                new Waypoint(112, 65, robot.theta + PI, -10, -20, 0, 1),
                                new Waypoint(96, 80, -PI/2, 10, 20, 0, parkTime),
                        };
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta + PI, -10, -20, 0, 0),
                                new Waypoint(76, 80, -PI/2, 10, 20, 0, parkTime),
                        };
                    } else {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta + PI, -10, -20, 0, 0),
                                new Waypoint(98, 80, -PI/2, 20, 50, 0, parkTime),
                        };
                    }
                    parkThetaSpline = new Spline(PI/4, 0, 0, -PI/2, 0, 0, parkTime);
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), parkThetaSpline.position(curTime) + PI);

                if (robot.isAtPose(curPose.getX(), curPose.getY(), PI/2)) {
                    robot.intake.sticksOut();
                    robot.wobbleArm.armDown();
                    park = true;
                }
            }

            else {
                robot.drivetrain.setControls(0, 0, 0);
            }

            robot.update();
        }

        robot.shooter.feedHome();

        robot.update();
        robot.stop();
    }
}
