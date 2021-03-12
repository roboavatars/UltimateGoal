package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.CombinedVision;
import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightPipeline.RingCase;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@Autonomous(name = "1 Red Auto", preselectTeleOp = "Teleop")
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
        robot.logger.startLogging(true);

        CombinedVision detector = new CombinedVision(this, CombinedVision.Pipeline.StackHeight);
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
        double intakeStackTime = 1.5;
        double shoot1RingTime = 0.75;
        double intakeStack2Time = 2.5;
        double goToPowerShootTime = 1.0;
        double shootPowerShotsTime = 3.0;
        double ringTime = 0;
        double deliverWobbleTime = 1.0;
        double intakeWobble2Time = 3.5;
        double goToHighShootTime = 1.0;
        double shootHighGoal2Time = 1.5;
        double deliverWobble2Time = 2.0;
        double parkTime = 1.5;

        // Paths
        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(114, 9, PI/2, 40, 50, 0, 0),
                new Waypoint(109, 33, PI/2, 40, 30, 0, goToStackTime),
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
        boolean sweep = true;
        ArrayList<Ring> rings;

        waitForStart();

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getStackPipe().getModeResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{120, 92}, {97, 114}, {123, 134}};
        double[][] wobble2Delivery = {{117, 83}, {94, 106}, {116, 128}};
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

        detector.setPipeline(CombinedVision.Pipeline.RingLocator);

        robot.intake.blockerDown();
        robot.intake.sticksHalf();
        robot.wobbleArm.clamp();

        ElapsedTime time = new ElapsedTime();

        robot.update();

        while (opModeIsActive() && System.currentTimeMillis() - robot.startTime < 30000) {

            rings = detector.getRingPipe().getRings(robot.x, robot.y, robot.theta);
            robot.ringPos = rings;

            // Go to starting stack
            if (!goToStack) {
                robot.setTargetPoint(goToStackPath.getRobotPose(Math.min(time.seconds(), goToStackTime)), PI/2, 0);

                robot.shooter.flywheelHG();

                if (time.seconds() > goToStackTime) {
                    robot.wobbleArm.setArmPosition(-250);

                    robot.shootYOverride = 33;
                    robot.thetaOffset = 0.03;
                    Constants.HIGH_GOAL_VELOCITY = 1760;
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
                        robot.highGoalShoot();
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
                    robot.setTargetPoint(108, input, PI/2);

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

                    robot.wobbleArm.setArmPosition(-300);

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
                        robot.intake.sticksHome();

                        Robot.log("Detected " + rings.size() + " ring(s): " + rings);

                        for (Ring ring : rings) {
                            sweep &= ring.getY() >= 138;
                        }

                        if (!sweep) {
                            ArrayList<Waypoint> ringWaypoints = new ArrayList<>();
                            ringWaypoints.add(new Waypoint(robot.x, robot.y, robot.theta, 50, 60, 0, 0));

                            double[] ringPos;
                            ringTime = 0;
                            if (rings.size() >= 1) {
                                ringPos = rings.get(0).driveToRing(robot.x, robot.y);
                                if (ringPos[1] > 135) {
                                    ringPos[2] = PI/2;
                                }
                                ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 20, 30, 0, ringTime + 1.5));
                                ringTime += 1.5;

                                if (rings.size() == 2) {
                                    ringPos = rings.get(1).driveToRing(ringPos[0], ringPos[1]);
                                    if (ringPos[1] > 135) {
                                        ringPos[2] = PI/2;
                                    }
                                    ringWaypoints.add(new Waypoint(ringPos[0], ringPos[1], ringPos[2], 20, 30, 0, ringTime + 1.5));
                                    ringTime += 1.5;
                                }
                            }
                            ringWaypoints.add(new Waypoint(123, 134, PI, 30, 20, 0, ringTime + 1.5));
                            ringTime += 1.5;
                            ringPath = new Path(ringWaypoints);
                        } else {
                            ringTime = 4.5;
                            Waypoint[] ringWaypoints = new Waypoint[] {
                                    new Waypoint(robot.x, robot.y, robot.theta, 50, 50, 0, 0),
                                    new Waypoint(60, 132, 0, 30, 30, 0, 1.5),
                                    new Waypoint(109, 132, 0, 30, 20, 0, 3.75),
                                    new Waypoint(123, 134, 0, 30, 5, 0, ringTime),
                            };
                            ringPath = new Path(new ArrayList<>(Arrays.asList(ringWaypoints)));
                        }

                        shootPowerShots = true;
                        time.reset();
                    }
                }
            }

            // Collect bounce backed rings
            else if (!bounceBack) {
                if (!sweep || time.seconds() < 1.5) {
                    robot.setTargetPoint(ringPath.getRobotPose(Math.min(time.seconds(), ringTime)));
                } else {
                    if (time.seconds() < 3.75) {
                        robot.setTargetPoint(new Target(ringPath.getRobotPose(Math.min(time.seconds(), ringTime))).thetaW0(PI/4).thetaKp(7.0));
                    } else {
                        robot.setTargetPoint(ringPath.getRobotPose(Math.min(time.seconds(), ringTime)), PI/2, 0);
                    }
                }

                if (time.seconds() > ringTime) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -40, -50, 0, 0),
                            new Waypoint(wobbleCor[0], wobbleCor[1], PI/2, -30, -30, 0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    bounceBack = true;
                    time.reset();
                }
            }

            // Deliver first wobble goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                if (ringCase != RingCase.Four && !sweep) {
                    Pose curPose = deliverWobblePath.getRobotPose(curTime);
                    robot.setTargetPoint(new Target(curPose).thetaW0(curPose.theta + PI).xyKd(0).thetaKd(0));
                }

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[1], wobbleCor[2], PI/2, 7, 7, 0.5))
                        || time.seconds() > 2) {
                    robot.wobbleArm.armDown();
                }

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], PI/2) && robot.wobbleArm.getPosition() < -400)
                        || time.seconds() > 2.5) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.unClamp();
                    robot.intake.reverse();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > 3) {
                    reachedDeposit = false;
                    depositReachTime = 0;

                    robot.intake.off();

                    Waypoint[] intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 0.1, 0.1, 0, 0),
                            new Waypoint(robot.x - 4, robot.y, PI/2, 0.1, 0.1, 0, 0.5),
                    };
                    Waypoint[] intakeWobble2Waypoints2 = new Waypoint[] {
                            new Waypoint(robot.x - 4, robot.y, PI/2, -5, -50, 0, 0),
                            new Waypoint(83, 27, PI/2, -0.1, 40, 0, 2.5),
                            new Waypoint(87, 27, PI/2, -0.1, -0.1, 0, intakeWobble2Time),
                    };

                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));
                    intakeWobble2Path2 = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints2)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to pick up second wobble goal
            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time + 0.5);
                if (curTime < 0.5) {
                    robot.setTargetPoint(intakeWobble2Path.getRobotPose(curTime), PI/2, 0);
                } else {
                    Pose curPose = intakeWobble2Path2.getRobotPose(curTime - 0.5);
                    if (curTime < 2.5) {
                        robot.setTargetPoint(curPose, curPose.theta + PI, 0);
                    } else {
                        robot.setTargetPoint(curPose, PI/2, 0);
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
                            new Waypoint(106, 63, PI/2, 30, 30, 0, goToHighShootTime),
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
                            new Waypoint(wobble2Cor[0], wobble2Cor[1], PI/2, 30, 30, 0, deliverWobble2Time),
                    };
                    deliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(deliverWobble2Waypoints)));

                    shootHighGoal2 = true;
                    time.reset();
                }
            }

            // Deliver second wobble goal
            else if (!deliverWobble2) {
                double curTime = Math.min(time.seconds(), deliverWobble2Time);
                robot.setTargetPoint(new Target(deliverWobble2Path.getRobotPose(curTime)).xyKd(0).thetaKd(0));

                if ((!reachedDeposit && robot.isAtPose(wobble2Cor[1], wobble2Cor[2], PI/2, 7, 7, 0.5))
                        || time.seconds() > 2) {
                    robot.wobbleArm.armDown();
                }

                if ((!reachedDeposit && robot.isAtPose(wobble2Cor[0], wobble2Cor[1], PI/2) && robot.wobbleArm.getPosition() < -400)
                        || time.seconds() > 2.5) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > 3) {

                    Waypoint[] parkWaypoints1 = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 0.1, 0.1, 0, 0),
                            new Waypoint(robot.x - 4, robot.y, PI/2, 0.1, 0.1, 0, 0.5),
                    };

                    Waypoint[] parkWaypoints2;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints2 = new Waypoint[] {
                                new Waypoint(robot.x - 4, robot.y, PI/2, 20, 20, 0, 0),
                                new Waypoint(89, 84, PI/2, 20, 10, 0, parkTime),
                        };
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints2 = new Waypoint[] {
                                new Waypoint(robot.x - 4, robot.y, PI/2, -30, -30, 0, 0),
                                new Waypoint(90, 84, PI/2, -30, -30, 0, parkTime),
                        };
                    } else {
                        parkWaypoints2 = new Waypoint[] {
                                new Waypoint(robot.x - 4, robot.y, PI/2, -50, -100, 0, 0),
                                new Waypoint(104, 86, PI/2, -30, -30, 0, parkTime),
                        };
                    }

                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints1)));
                    parkPath2 = new Path(new ArrayList<>(Arrays.asList(parkWaypoints2)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime + 0.5);
                if (curTime < 0.5) {
                    robot.setTargetPoint(parkPath.getRobotPose(curTime), PI/2, 0);
                } else {
                    Pose curPose = parkPath2.getRobotPose(curTime - 0.5);
                    if (ringCase == RingCase.Zero) {
                        robot.setTargetPoint(parkPath2.getRobotPose(curTime));
                    } else if (ringCase == RingCase.One) {
                        robot.setTargetPoint(curPose, curPose.theta + PI, 0);
                    } else {
                        if (curTime < 1) {
                            robot.setTargetPoint(curPose, 5*PI/4, 0);
                        } else {
                            robot.intake.sticksOut();
                        }
                    }
                }

                if (time.seconds() > parkTime) {
                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;
                }
            }

            else {
                robot.drivetrain.setControls(0, 0, 0);
                if (Math.abs(robot.vx) + Math.abs(robot.vy) < 1 && Math.abs(robot.w) < 0.5) {
                    break;
                }
            }

            addPacket("Ring Case", ringCase);
            robot.update();
        }

        robot.stop();
        try {
            detector.stop();
        } catch (Exception ignore) {}
    }
}
