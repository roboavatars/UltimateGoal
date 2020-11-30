package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.StackHeightDetector;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeightPipeline.RingCase;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Path;
import org.firstinspires.ftc.teamcode.Splines.Pose;
import org.firstinspires.ftc.teamcode.Splines.Spline;
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
        robot.intake.sticksHomeAuto();
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

        double startLineTime = 1.75;
        double shootPowerShotsTime = 4.0;
        double deliverWobbleTime = 2;
        double intakeWobble2Time = 3.75;
        double intakeStackTime = 4.0;
        double shootHighGoalTime = 4.0;
        double deliverWobble2Time = 3.5;
        double parkTime = 2.0;

        waitForStart();

        RingCase ringCase = detector.getResult();

        double[][] wobbleDelivery = {{123, 73}, {98, 97}, {126, 124}};
        double[][] wobble2Delivery = {{119, 68}, {96, 92}, {115, 120}};

        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Four) {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
            deliverWobbleTime += 2.5;
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];
        } else {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];
            deliverWobbleTime += 1;
            intakeStack = true;
        }

        detector.stop();

        if (ringCase == RingCase.Four) {
            robot.intake.rStickDown();
        }

        Waypoint[] startLineWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 40.0, 50.0, 0.0, 0.0),
                new Waypoint(90, 68, PI/2, 10.0, -10.0, 0.0, startLineTime),
        };
        Path startLinePath = new Path(new ArrayList<>(Arrays.asList(startLineWaypoints)));

        Path deliverWobblePath = null;
        Path intakeWobble2Path = null;
        Spline intakeWobble2ThetaSpline = null;
        Path intakeStackPath = null;
        Path deliverWobble2Path = null;
        Spline deliverWobble2ThetaSpline = null;
        Path parkPath = null;
        Spline parkThetaSpline = null;

        ElapsedTime time = new ElapsedTime();

        while(opModeIsActive()) {

            if (!startLine) {
                double curTime = Math.min(time.seconds(), startLineTime);
                Pose curPose = startLinePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                robot.shooter.flywheelPowershot();

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
                    Waypoint[] deliverWobbleWaypoints;
                    if (ringCase == RingCase.Zero) {
                        deliverWobbleWaypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 40.0, 50.0, 0.0, 0.0),
                                new Waypoint(wobbleCor[0], wobbleCor[1], PI/6, 50.0, -20.0, 0.0, deliverWobbleTime),
                        };
                    } else {
                        deliverWobbleWaypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 40.0, 50.0, 0.0, 0.0),
                                new Waypoint(wobbleCor[0], wobbleCor[1], PI/4, 50.0, -20.0, 0.0, deliverWobbleTime),
                        };
                    }
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    shootPowerShots = true;
                    time.reset();
                }
            }

            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > deliverWobbleTime + 1) {

                    robot.intake.sticksHome();
                    robot.wobbleArm.wobbleDown();
                    robot.shooter.magHome();

                    Waypoint[] intakeWobble2Waypoints;
                    if (ringCase == RingCase.Zero) {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                                new Waypoint(125, 66, PI/2, -20.0, 10.0, 0.0, 2.0),
                                new Waypoint(125, 37, 5*PI/12, 0.0, -50.0, 0.0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(5*PI/12, 0.4, 0.0, 5*PI/12, 0.0, 0.0, intakeWobble2Time);
                    } else if (ringCase == RingCase.One) {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                                new Waypoint(127, 66, PI/2, -15.0, 10.0, 0.0, 2.0),
                                new Waypoint(124, 37, 5*PI/12, 0.0, -30.0, 0.0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(5*PI/12, 0.2, 0.0, 5*PI/12, 0.0, 0.0, intakeWobble2Time);
                    } else {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, -10.0, -50.0, 0.0, 0.0),
                                new Waypoint(127, 66, PI/2, -30.0, -20.0, 0.0, 1.75),
                                new Waypoint(125, 37, 5*PI/12, 2.0, -20.0, 0.0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(PI/3, 0.2, 0.0, 5*PI/12, 0.0, 0.0, intakeWobble2Time);
                    }
                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));

                    deliverWobble = true;
                    time.reset();
                } else if (time.seconds() > deliverWobbleTime) {
                    robot.intake.setIntakePow(0.4);
                    robot.intake.sticksHalf();
                }
            }

            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time);
                Pose curPose = intakeWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), intakeWobble2ThetaSpline.position(curTime), 0.5, 0.5, 4.5);

                if (time.seconds() > intakeWobble2Time + 1.5) {
                    if (ringCase != RingCase.Zero) {
                        robot.intake.intakeOn();
                    } else {
                        robot.intake.intakeOff();
                    }

                    Waypoint[] intakeStackWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 20.0, 0.0, 0.0),
                            new Waypoint(111, 44, PI/2, 10.0, 0.0, 0.0, 1.5),
                            new Waypoint(109, 61, PI/2, 5.0, -20.0, 0.0, intakeStackTime),
                    };
                    intakeStackPath = new Path(new ArrayList<>(Arrays.asList(intakeStackWaypoints)));

                    if (ringCase != RingCase.Zero) {
                        robot.shooter.flywheelHighGoal();
                    }

                    intakeWobble2 = true;
                    time.reset();
                } else if (time.seconds() > intakeWobble2Time + 0.5) {
                    robot.wobbleArm.setWobbleMotorPosition(-300);
                } else if (time.seconds() > intakeWobble2Time - 0.5) {
                    robot.wobbleArm.wobbleClamp();
                } else if (time.seconds() > intakeWobble2Time - 1) {
                    robot.wobbleArm.wobbleRelease();
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
                    robot.intake.intakeOff();

                    Waypoint[] deliverWobble2Waypoints;
                    if (ringCase == RingCase.Zero) {
                        deliverWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 30.0, 0.0, 0.0),
                                new Waypoint(wobble2Cor[0], wobble2Cor[1], 5*PI/4, 10.0, -30.0, 0.0, deliverWobble2Time),
                        };
                        deliverWobble2ThetaSpline = new Spline(PI/2, 0.2, 0.0, 5*PI/4, 0.0, 0.0, deliverWobble2Time);
                    } else if (ringCase == RingCase.One) {
                        deliverWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 30.0, 0.0, 0.0),
                                new Waypoint(wobble2Cor[0], wobble2Cor[1], 5*PI/4, 10.0, -30.0, 0.0, deliverWobble2Time),
                        };
                        deliverWobble2ThetaSpline = new Spline(PI/2, 0.2, 0.0, 5*PI/4, 0.0, 0.0, deliverWobble2Time);
                    } else {
                        deliverWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta, 10.0, 30.0, 0.0, 0.0),
                                new Waypoint(wobble2Cor[0], wobble2Cor[1], 3*PI/2, 10.0, 20.0, 0.0, deliverWobble2Time),
                        };
                        deliverWobble2ThetaSpline = new Spline(PI/2, 0.2, 0.0, 5*PI/4, 0.0, 0.0, deliverWobble2Time);
                    }
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    shootHighGoal = true;
                    time.reset();
                }
            }

            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobble2Time);
                Pose curPose = deliverWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), deliverWobble2ThetaSpline.position(curTime));

                if (time.seconds() > deliverWobble2Time + 1.5) {
                    robot.wobbleArm.wobbleUp();

                    Waypoint[] parkWaypoints;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta + PI, -10.0, -20.0, 0.0, 0.0),
                                new Waypoint(121, 59, PI/3, 10.0, 20.0, 0.0, 1.0),
                                new Waypoint(98, 80, 3*PI/2, 10.0, 20.0, 0.0, parkTime),
                        };
                        parkThetaSpline = new Spline(PI/4, 0.0, 0.0, 3*PI/2, 0.0, 0.0, parkTime);
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta + PI, -10.0, -20.0, 0.0, 0.0),
                                new Waypoint(99, 84, PI/3, 10.0, 20.0, 0.0, 1.0),
                                new Waypoint(76, 81, 3*PI/2, 10.0, 20.0, 0.0, parkTime),
                        };
                        parkThetaSpline = new Spline(PI/4, 0.0, 0.0, 3*PI/2, 0.0, 0.0, parkTime);
                    } else {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.theta + PI, -10.0, -20.0, 0.0, 0.0),
                                new Waypoint(98, 80, 3*PI/2, 10.0, 20.0, 0.0, parkTime),
                        };
                        parkThetaSpline = new Spline(5*PI/4, 0.0, 0.0, 3*PI/2, 0.0, 0.4, parkTime);
                    }
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                } else if (time.seconds() > deliverWobble2Time) {
                    robot.wobbleArm.wobbleRelease();
                } else if (time.seconds() > deliverWobble2Time - 0.75) {
                    robot.wobbleArm.wobbleDown();
                }
            }

            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), parkThetaSpline.position(curTime) + PI);

                if (time.seconds() > 1) {
                    robot.wobbleArm.wobbleHome();
                    robot.shooter.feedHome();
                }

                if (robot.isAtPose(curPose.getX(), curPose.getY(), PI/2)) {
                    robot.intake.sticksOut();
                    park = true;
                }
            }

            else {
                robot.drivetrain.setControls(0, 0, 0);
            }

            robot.update();
        }

        robot.wobbleArm.wobbleHome();
        robot.shooter.feedHome();

        robot.update();
        robot.stop();
    }
}
