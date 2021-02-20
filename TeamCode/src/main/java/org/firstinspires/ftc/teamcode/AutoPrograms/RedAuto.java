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
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@Autonomous(name = "0 Red Auto", preselectTeleOp = "Teleop")
public class RedAuto extends LinearOpMode {

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
        robot.intake.blockerDown();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

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
        double shoot1RingTime = 0.75;
        double intakeStack2Time = 2.5;
        double goToPowerShootTime = 1.15;
        double shootPowerShotsTime = 3.0;
        double ringTime = 1.5;
        double deliverWobbleTime = 1.0;
        double intakeWobble2Time = 3.25;
        double goToHighShootTime = 0.75;
        double shootHighGoal2Time = 1.5;
        double deliverWobble2Time = 2.0;
        double parkTime = 1.75;

        // Paths
        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(114, 9, PI/2, 40, 50, 0, 0),
                new Waypoint(109, 38, PI/2, 40, 30, 0, goToStackTime),
        };
        Path goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));
        Path goToPowerShotPath = null;
        Path ringPath = null;
        Path deliverWobblePath = null;
        Path intakeWobble2Path = null;
        Path intakeWobble2Path2 = null;
        Path goToHighShootPath = null;
        Path deliverWobble2Path = null;
        Path parkPath = null;
        Path parkPath2 = null;

        // Other variables
        boolean shot1Ring = false;
        boolean psFinish = false;
        double psFinishTime = 0;
        boolean reachedDeposit = false;
        double depositReachTime = 0;
        boolean reachedWgPickup = false;
        double pickReachTime = 0;
        boolean visionClose = false;

        waitForStart();

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{116, 85}, {91, 106}, {116, 133}};
        double[][] wobble2Delivery = {{113, 80}, {91, 98}, {116, 127}};
        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];

            goToStack = true;
            shootHighGoal = true;
            intakeStack = true;
            goToPowerShootTime = 2.0;
            deliverWobbleTime = 2.0;
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];

            deliverWobbleTime = 1.5;
        } else {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
        }

        // make sure blocker is fully up so that cam can see the stack
        detector.stop();

//        RingLocator locator = new RingLocator(this);
        ArrayList<Ring> rings = new ArrayList<>();
        robot.ringPos = rings;

        robot.intake.blockerDown();
        robot.intake.leftHalf();
        robot.intake.sticksUpdate();
        robot.wobbleArm.clamp();

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            // Go to starting stack
            if (!goToStack) {
                robot.setTargetPoint(goToStackPath.getRobotPose(Math.min(time.seconds(), goToStackTime)), PI/2, 0);

                robot.shooter.flywheelHG();

                if (time.seconds() > goToStackTime) {
                    robot.wobbleArm.setArmPosition(-250);

                    robot.shootYOverride = 38;
                    robot.thetaOffset = 0.03;
                    Constants.HIGH_GOAL_VELOCITY = 1750;
                    robot.highGoalShoot(ringCase == RingCase.Four ? 3 : 1);

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
                robot.setTargetPoint(109, 43, PI/2);

                if (time.seconds() > intakeStackTime) {
                    if (ringCase == RingCase.Four) {
                        robot.shootYOverride = robot.y;
                        robot.highGoalShoot(1);
                    }

                    intakeStack = true;
                    time.reset();
                }
            }

            else if (!intakeStack2) {
                if (ringCase == RingCase.Four && !robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!shot1Ring) {
                        shot1Ring = true;
                        robot.intake.on();
                        time.reset();
                    }
                    double input = Math.min(60, 42 + 6 * time.seconds() + 1.5 * Math.sin(8.5 * time.seconds()));
                    robot.setTargetPoint(109, input, PI/2);

                    if (time.seconds() > intakeStack2Time) {
                        robot.shooter.flywheelPS();

                        Waypoint[] goToPowerShootWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 30, 20, 0, 0),
                                new Waypoint(87, 63, PI/2, 30, 30, 0, goToPowerShootTime),
                        };
                        goToPowerShotPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShootWaypoints)));

                        intakeStack2 = true;
                        time.reset();
                    }
                } else if (ringCase != RingCase.Four) {
                    robot.intake.off();
                    robot.shooter.flywheelPS();

                    Waypoint[] goToPowerShotWaypoints;
                    if (ringCase != RingCase.Zero) {
                        goToPowerShotWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 30, 20, 0, 0),
                                new Waypoint(87, 63, PI/2, 30, 30, 0, goToPowerShootTime),
                        };
                    } else {
                        goToPowerShotWaypoints = new Waypoint[] {
                                new Waypoint(114, 9, PI/2, 30, 30, 0, 0),
                                new Waypoint(114, 29, PI/2, 30, 30, 0, 1),
                                new Waypoint(87, 63, PI/2, 30, 30, 0, goToPowerShootTime),
                        };
                    }
                    goToPowerShotPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShotWaypoints)));

                    intakeStack2 = true;
                    time.reset();
                }
//                locator.start();
            }

            // Go to powershot shoot position
            else if (!goToPowerShoot) {
                robot.setTargetPoint(goToPowerShotPath.getRobotPose(Math.min(time.seconds(), goToPowerShootTime)));

                if (ringCase == RingCase.Four && time.seconds() > goToPowerShootTime - 0.15) {
                    robot.intake.reverse();
                }

                if (ringCase == RingCase.Zero && robot.y > 24) {
                    robot.wobbleArm.setArmPosition(-250);
                }

                if (time.seconds() > goToPowerShootTime) {
                    robot.thetaOffset = 0;
                    Constants.HIGH_GOAL_VELOCITY = 1850;
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
                        robot.drivetrain.setControls(0, 0, 0);
                    } else if (time.seconds() > psFinishTime + 0.5) {
                        robot.intake.on();

//                        rings = locator.getRings(robot.x, robot.y, robot.theta);
//                        Robot.log("Detected " + rings.size() + " ring(s): " + rings);
//                        if (rings.size() == 0) {
                            rings.add(new Ring(0, 0, 86, 125));
                            rings.add(new Ring(0, 0, 98, 120));
                            rings.add(new Ring(0, 0, 110, 115));
                            rings = Ring.getRingCoords(rings, robot.x, robot.y);
//                        }
                        try {
//                            locator.stop();
                            visionClose = true;
                        } catch (Exception ignore) {}

                        ArrayList<Waypoint> ringWaypoints = new ArrayList<>();
                        ringWaypoints.add(new Waypoint(robot.x, robot.y, robot.theta, 50, 60, 0, 0));

                        double[] ringPos = rings.get(0).driveToRing(robot.x, robot.y);
                        if (rings.size() >= 1) {
                            if (ringPos[1] > 135) {
                                ringPos[2] = PI/2;
                            }
                            ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 30, 30, 0, ringTime += 1.5));
                        }
                        if (rings.size() >= 2) {
                            ringPos = rings.get(1).driveToRing(ringPos[0], ringPos[1]);
                            if (ringPos[1] > 135) {
                                ringPos[2] = PI/2;
                            }
                            ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 30, 30, 0, ringTime += 1.0));
                        }
                        if (rings.size() == 3) {
                            ringPos = rings.get(2).driveToRing(ringPos[0], ringPos[1]);
                            if (ringPos[1] > 135) {
                                ringPos[2] = PI/2;
                            }
                            ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 30, 30, 0, ringTime += 1.0));
                        }
                        ringPath = new Path(ringWaypoints);

                        shootPowerShots = true;
                        time.reset();
                    }
                }
            }

            // Collect bounce backed rings
            else if (!bounceBack) {
                robot.setTargetPoint(ringPath.getRobotPose(Math.min(time.seconds(), ringTime)));

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
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), curPose.getTheta() + PI, curPose.getVx(), curPose.getVy(), 0, robot.drivetrain.xKp, robot.drivetrain.yKp, robot.drivetrain.thetaKp, 0, 0, 0);

                if (!reachedDeposit && Math.abs(robot.y - wobbleCor[1]) < 7 && Math.abs(PI - robot.theta) < 0.6) {
                    robot.wobbleArm.armDown();
                }

                if (!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], PI) && robot.wobbleArm.getPosition() < -400) {
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
                            new Waypoint(102, 41, 5*PI/12, -10, 0, 0, 2.5),
                            new Waypoint(100.5, 34, 5*PI/12, -0.1, 50, 0, intakeWobble2Time),
                    };
                    intakeWobble2Path2 = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints2)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to pick up second wobble goal
            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time + 0.5);
                if (curTime < 0.5) {
                    robot.setTargetPoint(intakeWobble2Path.getRobotPose(curTime));
                } else {
                    Pose curPose = intakeWobble2Path2.getRobotPose(curTime - 0.5);
                    if (robot.y > 41) {
                        robot.setTargetPoint(curPose, curPose.getTheta() + PI, 0);
                    } else {
                        robot.setTargetPoint(curPose, 5*PI/12, 0);
                    }
                }

                if (time.seconds() > intakeWobble2Time + 0.5) {
                    robot.wobbleArm.setArmPosition(-250);
                } else if (time.seconds() > intakeWobble2Time) {
                    robot.wobbleArm.clamp();
                }

                if (time.seconds() > intakeWobble2Time + 1.0) {
                    robot.shooter.flywheelHG();

                    Waypoint[] goToHighShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 40, 0, 0),
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

                if (time.seconds() > goToHighShootTime && Math.pow(robot.vx, 2) + Math.pow(robot.vy, 2) <= 3) {
                    robot.highGoalShoot(ringCase == RingCase.One ? 4 : 3);

                    goToHighShoot = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal2) {
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
                robot.setTargetPoint(deliverWobble2Path.getRobotPose(curTime), robot.drivetrain.xKp, robot.drivetrain.yKp, robot.drivetrain.thetaKp, 0, 0, 0);

                if (!reachedDeposit && Math.abs(robot.y - wobble2Cor[1]) < 7 && Math.abs(PI - robot.theta) < 0.6) {
                    robot.wobbleArm.armDown();
                }

                if (!reachedDeposit && robot.isAtPose(wobble2Cor[0], wobble2Cor[1], PI)
                        && robot.wobbleArm.getPosition() < -400) {
                    reachedDeposit = true;
                    robot.wobbleArm.unClamp();
                    depositReachTime = curTime;
                }

                if (reachedDeposit && time.seconds() > depositReachTime + 0.5) {

                    Waypoint[] parkWaypoints2;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints2 = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                                new Waypoint(98, 77, PI/2, 20, 10, 0, parkTime),
                        };
                    } else {
                        Waypoint[] parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                                new Waypoint(robot.x - 10, robot.y, PI/2, 5, -10, 0, 0.5),
                        };
                        parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                        if (ringCase == RingCase.One) {
                            parkWaypoints2 = new Waypoint[] {
                                    new Waypoint(robot.x - 10, robot.y, robot.theta, -30, -30, 0, 0),
                                    new Waypoint(82, 84, PI/2, -30, -30, 0, parkTime),
                            };
                        } else {
                            parkWaypoints2 = new Waypoint[] {
                                    new Waypoint(robot.x - 10, robot.y, robot.theta, -40, -40, 0, 0),
                                    new Waypoint(104, 86, PI/2, -30, -40, 0, parkTime),
                            };
                        }
                    }
                    parkPath2 = new Path(new ArrayList<>(Arrays.asList(parkWaypoints2)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime + 0.5);
                if (ringCase == RingCase.Zero) {
                    robot.setTargetPoint(parkPath2.getRobotPose(curTime));
                } else {
                    if (curTime < 0.5) {
                        robot.setTargetPoint(parkPath.getRobotPose(curTime));
                    } else {
                        Pose curPose = parkPath2.getRobotPose(curTime - 0.5);
                        robot.setTargetPoint(curPose, curPose.getTheta() + PI, 0);
                    }
                }

                if (time.seconds() > parkTime) {
                    robot.intake.sticksOut();

                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;

                    if (!visionClose) {
//                        locator.stop();
                    }
                }
            }

            else {
                robot.drivetrain.setControls(0, 0, 0);
            }

            addPacket("Ring Case", ringCase);
            addPacket("PS Finish", psFinish);
            addPacket("Shoot Power Shots", shootPowerShots);
            robot.update();
        }

        robot.shooter.feedHome();
        robot.stop();
    }
}
