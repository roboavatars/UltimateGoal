package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.OpenCV.RingLocator.RingLocator;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightDetector;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightPipeline.RingCase;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Spline;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.PI;

@Autonomous(name = "1 Red Auto", preselectTeleOp = "Teleop")
public class NewRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect stack
            shoot preloaded rings into high goal
            intake 3 rings from the stack
            shoot 3rd ring into hg
            intake 4th ring
            shoot rings at powershot
            detect rings that bounced back from powershot
            collect powershot bounce backs
            drop off wobble goal at corresponding zone
            go back to pick up second wobble
            shoot bounce back rings into high goal
            drop off second wobble at corresponding zone
            park on line
        */

        Robot robot = new Robot(this, 114, 9, PI/2, true);
        robot.logger.startLogging();
        robot.intake.sticksHome();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

        RingLocator locator = null;
        ArrayList<Ring> rings = new ArrayList<>();
        robot.ringPos = rings;

        // Segments
        boolean goToStack = false;
        boolean shootHighGoal = false;
        boolean intakeStack = false;
        boolean intakeStack2 = false;
        boolean goToPowerShoot = false;
        boolean shootPowerShots = false;
        boolean bounceBack = false;
        boolean deliverWobble = false;
        boolean intakeWobble2 = false;
        boolean goToHighShoot = false;
        boolean shootHighGoal2 = false;
        boolean deliverWobble2 = false;
        boolean park = false;

        // Segment Times
        double goToStackTime = 0.75;
        double shootHighGoal1Time = 1.5;
        double intakeStackTime = 1.25;
        double shootHighGoal1RingTime = 0.75;
        double intakeStack2Time = 2.5;
        double goToPowerShootTime = 1.0;
        double shootPowerShotsTime = 1.5;
        double bounceBackTime = 7.0, ringTime = 1.5;
        double deliverWobbleTime = 1.0;
        double intakeWobble2Time = 3.5;
        double goToHighShootTime = 0.5;
        double shootHighGoal2Time = 1.5;
        double deliverWobble2Time = 1.75;
        double parkTime = 1.75;

        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(114, 9, PI/2, 40, 50, 0, 0),
                new Waypoint(107, 38, PI/2, 40, 30, 0, goToStackTime),
        };
        Path goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));

        // Paths and Theta splines
        Path goToPowerShootPath = null;
        Path ringPath = null;
        Path deliverWobblePath = null;
        Path intakeWobble2Path = null;
        Path intakeWobble2Path2 = null;
        Path goToHighShootPath = null;
        Path deliverWobble2Path = null;
        Path parkPath = null;
        Spline parkThetaSpline = null;

        // Other variables
        boolean shotRing = false;
        boolean psFinish = false;
        double psFinishTime = 0;
        boolean reachedDeposit = false;
        double depositReachTime = 0;
        boolean reachedWgPickup = false;
        double pickReachTime = 0;

        waitForStart();

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getModeResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{121, 82}, {96, 103}, {119, 130}};
        double[][] wobble2Delivery = {{119, 74}, {96, 92}, {120, 127}};
        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];
        } else {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
        }

        detector.stop();

        robot.intake.blockerDown();
        robot.intake.leftHalf();
        robot.wobbleArm.clamp();

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            // Go to starting stack
            if (!goToStack) {
                robot.setTargetPoint(goToStackPath.getRobotPose(Math.min(time.seconds(), goToStackTime)), PI/2, 0);

                robot.shooter.flywheelHG();

                if (time.seconds() > goToStackTime) {

                    robot.wobbleArm.setArmPosition(-200);

                    robot.shootYOverride = 38;
                    robot.thetaOffset = 0.04;
                    robot.highGoalShoot(3);

                    goToStack = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    robot.intake.on();
                    robot.shooter.flywheelHG();

                    shootHighGoal = true;
                    time.reset();
                }
            }

            // Intake start stack
            else if (!intakeStack) {
                robot.setTargetPoint(108, 43, PI/2);

                if (time.seconds() > intakeStackTime) {
                    robot.shootYOverride = robot.y;
                    robot.highGoalShoot(1); // shoots 2 since after feed top it must go home, shooting another in the process

                    intakeStack = true;
                    time.reset();
                }
            }

            else if (!intakeStack2) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!shotRing) {
                        shotRing = true;
                        robot.thetaOffset = 0;
                        robot.intake.on();
                        time.reset();
                    }
                    double input = Math.min(60, 42 + 6 * time.seconds() + 1.5 * Math.sin(8.5 * time.seconds()));
                    robot.setTargetPoint(108, input, PI/2);
//                    robot.setTargetPoint(108, 65, PI/2);

                    if (time.seconds() > intakeStack2Time) {

//                        robot.intake.off();
                        robot.shooter.flywheelPS();

                        locator = new RingLocator(this);
                        locator.start();

                        Waypoint[] goToPowerShootWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 30, 20, 0, 0),
                                new Waypoint(87, 63, PI/2, 30, 30, 0, goToPowerShootTime),
                        };
                        goToPowerShootPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShootWaypoints)));

                        intakeStack2 = true;
                        time.reset();
                    }
                }
            }

            // Go to powershot shoot position
            else if (!goToPowerShoot) {
                robot.setTargetPoint(goToPowerShootPath.getRobotPose(Math.min(time.seconds(), goToPowerShootTime)));

                if (time.seconds() > goToPowerShootTime - 0.15) {
                    robot.intake.reverse();
                }

                if (time.seconds() > goToPowerShootTime) {
                    robot.powerShotShoot();

                    goToPowerShoot = true;
                    time.reset();
                }
            }

            // Time block to shoot powershot
            else if (!shootPowerShots) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!psFinish) {
                        psFinish = true;
                        psFinishTime = time.seconds();
                    } else if (time.seconds() > psFinishTime + 1) {
                        robot.intake.on();

                        rings = locator.getRings(robot.x, robot.y, robot.theta);
                        Robot.log("Detected " + rings.size() + " ring(s): " + rings);
                        if (rings.size() == 0) {
                            rings.add(new Ring(0, 0, 86, 125));
                            rings.add(new Ring(0, 0, 98, 120));
                            rings.add(new Ring(0, 0, 110, 115));
                        }
                        locator.stop();

                        ArrayList<Waypoint> ringWaypoints = new ArrayList<>();
                        ringWaypoints.add(new Waypoint(robot.x, robot.y, robot.theta, 50, 60, 0, 0));
                        if (rings.size() >= 1) {
                            ringTime += 1.5;
                            double[] ringPos = rings.get(0).driveToRing(robot.x, robot.y);
                            ringWaypoints.add(new Waypoint(ringPos[0] - 4, ringPos[1] - 4, PI/4, 30, 40, 0, 1.5));
                            ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] + 4, PI/4, 30, 40, 0, 2.0));
                            if (rings.size() >= 2) {
                                double ringY = rings.get(1).driveToRing(90, 33)[1];
                                ringWaypoints.add(new Waypoint(ringPos[0], ringY - 15, PI/2, -30, -10, 0, 3.0));
                            } else {
                                ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] - 10, PI/2, -30, -10, 0, 3.0));
                            }
                        }
                        if (rings.size() >= 2) {
                            ringTime += 2.5;
                            double[] ringPos = rings.get(1).driveToRing(robot.x, robot.y);
                            ringWaypoints.add(new Waypoint(ringPos[0] - 4, ringPos[1] - 4, PI/4, 30, 40, 0, 4.0));
                            ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] + 4, PI/4, 30, 40, 0, 4.5));
                            if (rings.size() == 3) {
                                double ringY = rings.get(2).driveToRing(robot.x, 33)[1];
                                ringWaypoints.add(new Waypoint(ringPos[0], ringY - 15, PI/2, -30, -10, 0, 5.5));
                            } else {
                                ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] - 10, PI/2, -30, -10, 0, 5.5));
                            }
                        }
                        if (rings.size() == 3) {
                            ringTime += 1.5;
                            double[] ringPos = rings.get(2).driveToRing(robot.x, robot.y);
                            ringWaypoints.add(new Waypoint(ringPos[0] - 4, ringPos[1] - 4, PI/4, 30, 40, 0, 6.5));
                            ringWaypoints.add(new Waypoint(ringPos[0] + 4, ringPos[1] + 4, PI/4, 30, 40, 0, ringTime));
                        }
                        ringPath = new Path(ringWaypoints);

                        shootPowerShots = true;
                        time.reset();
                    }
                }
            }

            // Collect bounce backed rings
            else if (!bounceBack) {
                double curTime = Math.min(time.seconds(), ringTime);
                if ((1.5 < ringTime && ringTime < 2.0) || (4.0 < ringTime && ringTime < 4.5) || (ringTime > 6.5)) {
                    robot.setTargetPoint(ringPath.getRobotPose(curTime), PI/4, 0, 0.1);
                } else {
                    robot.setTargetPoint(ringPath.getRobotPose(curTime), PI/2, 0, 0.1);
                }

                if (time.seconds() > ringTime) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -40, -50, 0, 0),
                            new Waypoint(wobbleCor[0], wobbleCor[1], PI, -30, -30, 0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    bounceBack = true;
                    time.reset();
                }
            }

            // Deliver first wobble goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                robot.setTargetPoint(deliverWobblePath.getRobotPose(curTime), PI, 0);

                if (!reachedDeposit && Math.abs(robot.y - wobbleCor[1]) < 5 && Math.abs(PI - robot.theta) < 0.5) {
                    robot.wobbleArm.armDown();
                }

                if (!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], PI, 2, 2, PI/8)) {
                    reachedDeposit = true;
                    robot.wobbleArm.unClamp();
                    depositReachTime = curTime;
                }

                if (reachedDeposit && time.seconds() > depositReachTime + 0.5) {
                    reachedDeposit = false;
                    depositReachTime = 0;

                    robot.intake.off();

                    Waypoint[] intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                            new Waypoint(robot.x - 12, robot.y, 3*PI/4, 5, -10, 0, 0.5),
                    };
                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));

                    Waypoint[] intakeWobble2Waypoints2 = new Waypoint[] {
                            new Waypoint(robot.x - 12, robot.y, 3*PI/4, -5, -50, 0, 0),
                            new Waypoint(102.5, 36, 5*PI/12, 5, 100, 0, 3.5),
                    };
                    intakeWobble2Path2 = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints2)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to pick up second wobble goal
            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time);
                if (curTime < 0.5) {
                    robot.setTargetPoint(intakeWobble2Path.getRobotPose(curTime));
                } else {
                    Pose curPose = intakeWobble2Path2.getRobotPose(curTime);
                    robot.setTargetPoint(curPose, 5*PI/12, 0);
                }

                if (time.seconds() > intakeWobble2Time) {
                    robot.wobbleArm.clamp();
                }

                if (time.seconds() > intakeWobble2Time + 0.25) {
                    robot.wobbleArm.armUp();
                    robot.shooter.flywheelHG();

                    Waypoint[] goToHighShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 50, 0, 0),
                            new Waypoint(106, 68, PI/2, 30, 30, 0, goToHighShootTime),
                    };
                    goToHighShootPath = new Path(new ArrayList<>(Arrays.asList(goToHighShootWaypoints)));

                    intakeWobble2 = true;
                    time.reset();
                }
            }

            // Go to high goal shoot position
            else if (!goToHighShoot) {
                robot.setTargetPoint(goToHighShootPath.getRobotPose(Math.min(time.seconds(), goToHighShootTime)));

                if (time.seconds() > goToHighShootTime) {
                    robot.highGoalShoot(3);

                    goToHighShoot = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal2) {
                robot.wobbleArm.armDown();

                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 40, 0, 0),
                            new Waypoint(wobble2Cor[0], wobble2Cor[1], PI, 30, 30, 0, deliverWobble2Time),
                    };
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    shootHighGoal2 = true;
                    time.reset();
                }
            }

            // Deliver second wobble goal
            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobble2Time);
                robot.setTargetPoint(deliverWobble2Path.getRobotPose(curTime));

                if (!reachedDeposit && Math.abs(robot.y - wobble2Cor[1]) < 5 && Math.abs(PI - robot.theta) < 0.5) {
                    robot.wobbleArm.armDown();
                }

                if (!reachedDeposit && robot.isAtPose(wobble2Cor[0], wobble2Cor[1], PI, 3, 3, PI/7)) {
                    reachedDeposit = true;
                    robot.wobbleArm.unClamp();
                    depositReachTime = curTime;
                }

                if (reachedDeposit && time.seconds() > depositReachTime + 0.5) {

                    Waypoint[] parkWaypoints;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -20, -20, 0, 0),
                                new Waypoint(112, 65, robot.theta, -20, -20, 0, 1),
                                new Waypoint(94, 80, PI/2, 30, 20, 0, parkTime),
                        };
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
                                new Waypoint(76, 80, PI/2, 30, 20, 0, parkTime),
                        };
                    } else {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -40, -50, 0, 0),
                                new Waypoint(98, 86, PI/2, -30, -30, 0, parkTime),
                        };
                    }
//                    parkThetaSpline = new Spline(robot.theta, 0, 0, PI/2, 0, 0, parkTime);
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                robot.setTargetPoint(parkPath.getRobotPose(curTime));

                if (time.seconds() > parkTime) {
                    robot.intake.sticksOut();
//                    robot.wobbleArm.down();

                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;
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
