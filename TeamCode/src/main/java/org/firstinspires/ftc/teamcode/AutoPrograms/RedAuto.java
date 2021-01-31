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

@Autonomous(name = "OldRedAuto")
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
            detect rings that bounced back from powershot
            shoot stack rings into high goal
            drop off second wobble at corresponding zone
            intake and shoot bounced off rings
            park
        */

        Robot robot = new Robot(this, 90, 9, PI/2, true);
        robot.logger.startLogging();
        robot.intake.sticksHome();

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
        boolean shootHighGoal2 = true;
        boolean park = false;

        // Segment Times
        double startLineTime = 1.5;
        double shootPowerShotsTime = 3.0;
        double deliverWobbleTime = 1.5;
        double intakeWobble2Time = 5.0;
        double intakeStackTime = 3.0;
        double shootHighGoalTime = 2.0;
        double deliverWobble2Time = 2.0;
        double shootHighGoal2Time = 2.0;
        double parkTime = 1.5;
        // 4 extra seconds for wobble related tasks

        waitForStart();

        robot.intake.blockerDown();
        robot.intake.leftHalf();
        robot.wobbleArm.setArmPosition(-120);

        sleep(300);

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getModeResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{121, 82}, {96, 103}, {125, 125}};
        double[][] wobble2Delivery = {{119, 74}, {96, 92}, {121, 122}};
        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];
            intakeWobble2Time = 3.0;
            intakeStack = true;
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];
            shootHighGoalTime = 2.0;
        } else {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
            intakeStackTime = 1.25;
            deliverWobbleTime = 2.0;
        }
        boolean hgFinish = false;
        double hgFinishTime = 0;
        boolean reached = false;
        double reachedTime = 0;
        boolean doneShooting = false;

        detector.stop();

        Waypoint[] startLineWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 60, 60, 0, 0),
                new Waypoint(87, 60, PI/2, 30, -30, 0, startLineTime),
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
        Path shootHighGoal2Path = null;
        Spline shootHighGoal2ThetaSpline = null;
        Path parkPath = null;
        Spline parkThetaSpline = null;

        ElapsedTime time = new ElapsedTime();

        robot.wobbleArm.clampWobble();

        while(opModeIsActive()) {

            // Go to shooting line to shoot powershots
            if (!startLine) {
                double curTime = Math.min(time.seconds(), startLineTime);
                Pose curPose = startLinePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > startLineTime - 1.25) {
//                    robot.shooter.flywheelPS();
                    //robot.shooter.flywheelHG();
                    robot.wobbleArm.setArmPosition(-300);
                }

                if (time.seconds() > startLineTime) {
//                    robot.powerShotShoot();
                    robot.highGoalShoot();

                    startLine = true;
                    time.reset();
                }
            }

            // Time block to shoot powershots
            else if (!shootPowerShots) {
                double curTime = time.seconds();
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!hgFinish) {
                        hgFinish = true;
                        hgFinishTime = curTime;
                    } else if (curTime > hgFinishTime + 0.2) {
                        Waypoint[] deliverWobbleWaypoints;
                        if (ringCase == RingCase.Zero) {
                            deliverWobbleWaypoints = new Waypoint[] {
                                    new Waypoint(robot.x, robot.y, robot.theta, 30, 30, 0, 0),
                                    new Waypoint(wobbleCor[0], wobbleCor[1], 13*PI/12, 10, -20, 0, deliverWobbleTime),
                            };
                        } else {
                            deliverWobbleWaypoints = new Waypoint[] {
                                    new Waypoint(robot.x, robot.y, robot.theta, 50, 40, 0, 0),
                                    new Waypoint(wobbleCor[0], wobbleCor[1], 13*PI/12, 40, -20, 0, deliverWobbleTime),
                            };
                        }
//                        deliverWobbleThetaSpline = new Spline(robot.theta, 0.3, 0, 13*PI/12, 0, 0, deliverWobbleTime);
                        deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                        shootPowerShots = true;
                        time.reset();
                    }
                }
            }

            // Deliver first wobble goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                if (robot.y < 110) {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), PI/4);
                } else {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), 13*PI/12);
                }

                if (time.seconds() > deliverWobbleTime + 1) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (time.seconds() > deliverWobbleTime + 0.75) {
                    robot.wobbleArm.unClampWobble();
                } else if (time.seconds() > deliverWobbleTime + 0.25) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > deliverWobbleTime + 1.25) {
                    Waypoint[] intakeWobble2Waypoints;
                    if (ringCase == RingCase.Zero) {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -10, -50, 0, 0),
                                new Waypoint(wobbleCor[0] - 4, wobbleCor[1] - 5, robot.theta, -20, -40, 0, 0.25),
                                new Waypoint(127, 63, PI/2, -20, -5, 0, 1),
                                new Waypoint(124, 36.5, 5*PI/12, 0, 30, 0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(robot.theta, -8, 0, 5*PI/12, 0, 0, intakeWobble2Time);
                    } else if (ringCase == RingCase.One) {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -10, -50, 0, 0),
                                new Waypoint(wobbleCor[0] - 4, wobbleCor[1] - 5, robot.theta, -20, -40, 0, 0.25),
                                new Waypoint(127, 66, PI/2, -30, -5, 0, 1.5),
                                new Waypoint(124, 36.5, 5*PI/12, 0, 30, 0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(robot.theta, -8, 0, 5*PI/12, 0, 0, intakeWobble2Time);
                    } else {
                        intakeWobble2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -30, -50, 0, 0),
                                new Waypoint(wobbleCor[0] - 4, wobbleCor[1] - 5, robot.theta, -20, -40, 0, 0.25),
                                new Waypoint(128, 66, PI/2, -30, -5, 0, 2),
                                new Waypoint(124, 36.5, 5*PI/12, 0, 60, 0, intakeWobble2Time),
                        };
                        intakeWobble2ThetaSpline = new Spline(robot.theta, -8, 0, 5*PI/12, 0, 0, intakeWobble2Time);
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
                if (ringCase == RingCase.Four && robot.y > 55 && robot.y < 115) {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), PI/2, 0.50, 0.25, 2.4);
                } else {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), intakeWobble2ThetaSpline.position(curTime), 0.50, 0.25, 1.5);
                }

                if (!reached && robot.isAtPose(124, 36.5, 5*PI/12, 0.4, 0.4, PI/35)) {
                    reached = true;
                    reachedTime = curTime;
                }

                if (reached && time.seconds() > reachedTime + 1) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (reached && time.seconds() > reachedTime + 0.5) {
                    robot.wobbleArm.clampWobble();
                } else if (time.seconds() > intakeWobble2Time - 1.5) {
                    robot.wobbleArm.unClampWobble();
                } else if (time.seconds() > intakeWobble2Time - 2) {
                    robot.wobbleArm.armDown();
                }

                if (reached && time.seconds() > reachedTime + 1.5) {
                    if (ringCase != RingCase.Zero) {
                        robot.intake.intakeOn();
                        //robot.shooter.flywheelHG();
                    }

                    Waypoint[] intakeStackWaypoints;
                    if (ringCase == RingCase.One) {
                        intakeStackWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -10, -20, 0, 0),
                                new Waypoint(107, 53, 7*PI/12, 30, 20, 0, intakeStackTime),
                        };
                        intakeStackThetaSpline = new Spline(robot.theta, 3, 0, 7*PI/12, 0, 0, intakeStackTime);
                    } else {
                        intakeStackWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -10, -20, 0, 0),
                                new Waypoint(robot.x - 13, robot.y - 6, 7*PI/12, 30, 15, 0, 0.5),
                                new Waypoint(109, 39, PI/2, 30, 15, 0, intakeStackTime),
                        };
                        intakeStackThetaSpline = new Spline(robot.theta, 1, 0, PI/2, 0, 0, intakeStackTime);
                    }
                    intakeStackPath = new Path(new ArrayList<>(Arrays.asList(intakeStackWaypoints)));

                    intakeWobble2 = true;
                    time.reset();
                }
            }

            // Intake start stack
            else if (!intakeStack) {
                double curTime = Math.min(time.seconds(), intakeStackTime);
                Pose curPose = intakeStackPath.getRobotPose(curTime);
                if (ringCase != RingCase.Four || robot.y < 40) {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), intakeStackThetaSpline.position(curTime));
                } else {
                    double input = Math.min(72, 39 + 4 * time.seconds() + 1.5 * Math.sin(12 * time.seconds()));
                    robot.setTargetPoint(109, input, Math.PI / 2, 0.2, 0.15, 1.2);
                }

                if (time.seconds() > intakeStackTime + 1.25) {
                    //robot.intake.intakeRev();
                }

                if ((ringCase != RingCase.Four && time.seconds() > intakeStackTime) || (ringCase == RingCase.Four && time.seconds() > intakeStackTime + 1.5)) {
                    robot.highGoalShoot();

                    intakeStack = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    robot.intake.intakeOn();

                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 50, 40, 0, 0),
                            new Waypoint(wobble2Cor[0], wobble2Cor[1], 5*PI/4, 30, -20, 0, deliverWobble2Time),
                    };
//                    deliverWobble2ThetaSpline = new Spline(robot.theta, 0.3, 0, 5*PI/4, 0, 0, deliverWobble2Time);
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    shootHighGoal = true;
                    time.reset();
                }
            }

            // Deliver second wobble goal
            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobble2Time);
                Pose curPose = deliverWobble2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta());

                if (time.seconds() > deliverWobble2Time + 1) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (time.seconds() > deliverWobble2Time + 0.75) {
                    robot.wobbleArm.unClampWobble();
                } else if (time.seconds() > deliverWobble2Time + 0.25) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > deliverWobble2Time + 1.25) {
                    robot.wobbleArm.clampWobble();
                    robot.intake.intakeOff();

                    if (ringCase == RingCase.Four) {
                        shootHighGoal2 = false;

                        //robot.shooter.flywheelHG();

                        Waypoint[] shootHighGoal2Waypoints;
                        shootHighGoal2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -40, -40, 0, 0),
                                new Waypoint(robot.x - 10, robot.y - 10, robot.theta, -40, -20, 0, 0.4),
                                new Waypoint(80, 60, PI/3, -30, 50, 0, shootHighGoal2Time),
                        };
//                        shootHighGoal2ThetaSpline = new Spline(robot.theta, -10, 0, PI/2, 0, 0, shootHighGoal2Time);
                        shootHighGoal2Path = new Path(new ArrayList<>(Arrays.asList(shootHighGoal2Waypoints)));
                    } else {
                        Waypoint[] parkWaypoints;
                        if (ringCase == RingCase.Zero) {
                            parkWaypoints = new Waypoint[] {
                                    new Waypoint(robot.x, robot.y, robot.theta, -20, -20, 0, 0),
                                    new Waypoint(112, 65, robot.theta, -20, -20, 0, 1),
                                    new Waypoint(94, 80, PI/2, 30, 20, 0, parkTime),
                            };
                        } else {
                            parkWaypoints = new Waypoint[] {
                                    new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
                                    new Waypoint(76, 80, PI/2, 30, 20, 0, parkTime),
                            };
                        }
                        parkThetaSpline = new Spline(5*PI/4, 0, 0, PI/2, 0, 0, parkTime);
                        parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));
                    }

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Shoot remaining rings
            else if (!shootHighGoal2) {
                double curTime = Math.min(time.seconds(), shootHighGoal2Time);
                Pose curPose = shootHighGoal2Path.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), PI/3);

                if (robot.isAtPose(80,60,Math.PI/3)) {
                    Waypoint[] parkWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 50, 0, 0),
                            new Waypoint(robot.x, 83, PI/2, 30, -30, 0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    robot.wobbleArm.armDown();
                    robot.intake.intakeOff();
                    robot.highGoalShoot();
                    shootHighGoal2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!doneShooting) {
                        doneShooting = true;
                        time.reset();
                    }

                    double curTime = Math.min(time.seconds(), parkTime);
                    Pose curPose = parkPath.getRobotPose(curTime);
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), ringCase == RingCase.Four ? PI/2 : parkThetaSpline.position(curTime));

                    if (time.seconds() > parkTime) {
                        robot.intake.sticksOut();
                        robot.wobbleArm.armDown();

                        park = true;
                    }
                }
            }

            else {
                robot.drivetrain.setControls(0, 0, 0);
            }

            robot.update();
        }

        robot.shooter.feedHome();
        robot.stop();
    }
}
