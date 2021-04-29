package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightPipeline.RingCase;
import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

@Autonomous(name = "1 Red Auto", preselectTeleOp = "1 Teleop")
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

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        // Segments
        boolean goToStack = false;
        boolean shootHighGoal = false;
        boolean intakeStack = false;
        boolean intakeStack2 = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean bounceBack = false;
        boolean intakeWobble2 = false;
        boolean goToHighShoot = false;
        boolean shootHighGoal2 = false;
        boolean deliverWobble2 = false;
        boolean park = false;

        // Segment Times
        double goToStackTime = 0.75;
        double shootHighGoal1Time = 1.5;
        double intakeStackTime = 2.5;
        double shoot1RingTime = 0.75;
        double intakeStack2Time = 2.25;
        double shootPowerShotsTime = 3.0;
        double deliverWobbleTime = 1.75;
        double ringTime = 0;
        double intakeWobble2Time = 3.25;
        double goToHighShootTime = 0.75;
        double shootHighGoal2Time = 1.5;
        double deliverWobble2Time = 2.25;
        double parkTime = 1.5;

        // Paths
        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(114, 9, PI/2, 30, 30, 0, 0),
                new Waypoint(109, 32, PI/2, 5, -30, 0, goToStackTime),
        };
        Path goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));
        Path intakeStack2Path = null;
        Path deliverWobblePath = null;
        Path ringPath = null;
        Path intakeWobble2Path = null;
        Path goToHighShootPath = null;
        Path deliverWobble2Path = null;
        Path parkPath = null;

        // Other variables
        final int highGoalVelocity = Constants.HIGH_GOAL_VELOCITY;
        final double flapUpPosition = Constants.FLAP_UP_POS;
        boolean knockStack = false;
        boolean shot1Ring = false;
        boolean psFinish = false;
        double psFinishTime = 0;
        boolean reachedDeposit = false;
        double depositReachTime = 0;
        boolean sweep = true;
        double[] ringIntakeTheta = new double[3];
        ArrayList<Ring> rings;

        waitForStart();

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getStackPipe().getModeResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{121, 93}, {97, 116}, {124.5, 128}};
        double[][] wobble2Delivery = {{116, 79}, {92, 108}, {115.5, 127.5}};
        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];

            goToStackTime = 1.75;
            goToStackWaypoints = new Waypoint[] {
                    new Waypoint(114, 9, PI/2, 40, 50, 0, 0),
                    new Waypoint(111, 63, PI/2, 30, 20, 0, goToStackTime),
            };
            goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));

            shootHighGoal = true;
            intakeStack = true;
            intakeStack2 = true;
            parkTime = 2.0;
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];
        } else {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
            intakeStack2Time = 2.0;
        }

        detector.setPipeline(Vision.Pipeline.RingLocator);
        robot.intake.sticksHalf();

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            rings = detector.getRingPipe().getRings(robot.x, robot.y, robot.theta);
            robot.ringPos = rings;

            // Go to starting stack
            if (!goToStack) {
                robot.setTargetPoint(new Target(goToStackPath.getRobotPose(Math.min(time.seconds(), goToStackTime))).thetaW0(PI/2).xKp(0.6));

                if (time.seconds() > 0.25) {
                    robot.intake.blockerDown();
                }

                if (ringCase != RingCase.Zero) {
                    Constants.HIGH_GOAL_VELOCITY = Constants.HIGH_GOAL_BACK_VELOCITY;
                    Constants.FLAP_UP_POS = Constants.FLAP_BACK_POS;
                    robot.shooter.flywheelHG();
                } else {
                    robot.shooter.flywheelPS();
                }

                if (time.seconds() > goToStackTime) {
                    if (ringCase != RingCase.Zero) {
                        robot.shootYOverride = 28;
                        robot.thetaOffset = 0.08;
                        robot.highGoalShoot(ringCase == RingCase.Four ? 4 : 1, true);
                    } else {
                        robot.powerShotShoot();
                    }

                    goToStack = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (ringCase == RingCase.Four) {
                        robot.shooter.flywheelHG();
                        robot.intake.stackOut();
                    }

                    shootHighGoal = true;
                    time.reset();
                } else {
                    if (time.seconds() > 0.5 && ringCase == RingCase.Four) {
                        robot.intake.stackOut();
                    }
                }
            }

            // Intake start stack
            else if (!intakeStack) {
                if (ringCase == RingCase.Four) {

                    if (time.seconds() > intakeStackTime - 0.25) {
                        robot.intake.setPower(-0.5);
                    } else if (knockStack) {
                        robot.intake.on();
                        robot.setTargetPoint(new Target(110, Math.min(36 + 4.5 * time.seconds(), 41), PI/2).thetaW0(PI/2).thetaKp(5.0));
                    } else if (robot.isAtPose(110, 36, PI/2, 0.5, 0.5, PI/35) && robot.notMoving()) {
                        robot.drivetrain.stop();
                        knockStack = true;
                    } else {
                        robot.setTargetPoint(110, 36, PI/2);
                    }
                }

                if (time.seconds() > intakeStackTime || ringCase == RingCase.One) {
                    if (ringCase == RingCase.Four) {
                        robot.shootYOverride = robot.y;
                        robot.highGoalShoot(1, true);
//                        robot.intake.stackHome();
                    }

                    intakeStack = true;
                    time.reset();
                }
            }

            else if (!intakeStack2) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!shot1Ring) {
                        shot1Ring = true;
                        robot.intake.on();
                        time.reset();
                    }

                    if (time.seconds() < 1.5) {
                        robot.setTargetPoint(new Target(109, 64, PI/2).thetaW0(PI/2).thetaKp(5.0));
                    } else {
                        robot.setTargetPoint(111, 63, PI/2);
                    }

                    if (time.seconds() > intakeStack2Time - 1) {
                        robot.shooter.flywheelPS();
                    }

                    if (time.seconds() > intakeStack2Time) {
                        robot.intake.stackHome();

                        robot.thetaOffset = 0;
                        Constants.HIGH_GOAL_VELOCITY = highGoalVelocity;
                        Constants.FLAP_UP_POS = flapUpPosition;
                        robot.powerShotShoot();

                        intakeStack2 = true;
                        time.reset();
                    }
                }
            }

            // Time block to shoot powershot
            else if (!shootPowerShots) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!psFinish) {
                        psFinish = true;
                        psFinishTime = time.seconds();
                        robot.drivetrain.stop();
                    } else if (time.seconds() > psFinishTime + (ringCase != RingCase.Four ? 2 : 0.5)) {
                        robot.intake.on();
                        robot.intake.sticksHome();

                        Robot.log("Detected " + rings.size() + " ring(s): " + rings);

//                        int maxY = 130;
                        sweep = true;
                        Ring closestRing = null;
                        for (Ring ring : rings) {
                            if (ring.getX() > 72 && ring.getY() > 96 && ring.getY() < 126) {
                                sweep = false;
                                closestRing = ring.clone();
                                break;
                            }
                        }

                        /*if (!sweep) {
                            ArrayList<Waypoint> ringWaypoints = new ArrayList<>();
                            ringWaypoints.add(new Waypoint(robot.x, robot.y, robot.theta, 50, 60, 0, 0));

                            double[] ringPos = new double[3];
                            ringTime = 0;
                            if (rings.size() >= 1) {
                                ringPos = rings.get(0).driveToRing(robot.x, robot.y);
                                ringTime += 1.5;
                                if (ringPos[1] > maxY) {
                                    ringIntakeTheta[0] = ringPos[0] - robot.x < 0 ? 3*PI/4 : PI/4;
                                    ringPos[0] -= 2;
                                } else if (Ring.isLowestX(rings, rings.get(0))) {
                                    ringIntakeTheta[0] = PI/2;
                                } else {
                                    ringIntakeTheta[0] = ringPos[2];
                                }
                                ringWaypoints.add(new Waypoint(ringPos[0], Math.min(maxY, ringPos[1]), ringIntakeTheta[0], 30, 10, 0, ringTime));
                            }

                            if (rings.size() >= 2) {
                                ringPos = rings.get(1).driveToRing(ringPos[0], Math.min(maxY, ringPos[1]));
                                ringTime += 1.5;
                                if (ringPos[1] > maxY || Ring.isLowestX(rings, rings.get(1))) {
                                    if (ringPos[1] > maxY && !Ring.isLowestX(rings, rings.get(1))) {
                                        ringIntakeTheta[1] = ringPos[0] - rings.get(0).getX() < 0 ? 3 * PI / 4 : PI / 4;
                                        ringPos[0] -= 2;
                                    } else {
                                        ringIntakeTheta[1] = PI/4;
                                        ringWaypoints.add(new Waypoint(ringPos[0]-5, Math.min(maxY, ringPos[1])-5, ringIntakeTheta[1], 30, 10, 0, ringTime-0.6));
                                    }
                                } else {
                                    ringIntakeTheta[1] = ringPos[2];
                                }
                                ringWaypoints.add(new Waypoint(ringPos[0], Math.min(maxY, ringPos[1]), ringIntakeTheta[1], 30, 10, 0, ringTime));
                            }

                            if (rings.size() >= 3) {
                                ringPos = rings.get(2).driveToRing(ringPos[0], Math.min(maxY, ringPos[1]));
                                ringTime += 1.5;
                                if (ringPos[1] > maxY && rings.get(2).getY() > maxY) {
                                    ringIntakeTheta[2] = ringPos[0] - rings.get(1).getX() < 0 ? 3*PI/4 : PI/4;
                                    ringPos[0] -= 2;
                                } else {
                                    ringIntakeTheta[2] = ringPos[2];
                                }
                                ringWaypoints.add(new Waypoint(ringPos[0], Math.min(maxY, ringPos[1]), ringIntakeTheta[2], 30, 10, 0, ringTime));
                            }
                            ringPath = new Path(new ArrayList<>(ringWaypoints));
                        } else {*/
                        ringTime = 5.0;
                        ArrayList<Waypoint> ringWaypoints = new ArrayList<>();
                        ringWaypoints.add(new Waypoint(robot.x, robot.y, robot.theta, 60, 60, 0, 0));
                        if (!sweep) {
                            ringWaypoints.add(new Waypoint(closestRing.getX(), closestRing.getY(), closestRing.driveToRing(robot.x, robot.y)[2], 40, 30, 0, 1.25));
                        }
                        ringWaypoints.add(new Waypoint(sweep ? 62 : 65, 127, 0, 40, 30, 0, sweep ? 2.0 : 2.25));
                        ringWaypoints.add(new Waypoint(116, 127, 0, 40, 30, 0, 3.75));
                        ringWaypoints.add(new Waypoint(wobbleDelivery[2][0], wobbleDelivery[2][1], 0, 40, 5, 0, ringTime));
                        ringPath = new Path(ringWaypoints);
//                        }

                        shootPowerShots = true;
                        time.reset();

                        // break;
                    }
                }
            }

            // Collect bounce backed rings
            else if (!bounceBack) {
//                if (!sweep) {
//                    if (time.seconds() < 0.75) {
//                        robot.setTargetPoint(ringPath.getRobotPose(Math.min(time.seconds(), ringTime)));
//                    } else if (rings.size() >= 1 && time.seconds() < 1.5) {
//                        robot.setTargetPoint(new Target(ringPath.getRobotPose(Math.min(time.seconds(), ringTime))).thetaW0(ringIntakeTheta[0]));
//                    } else if (rings.size() >= 2 && time.seconds() < 3.0) {
//                        robot.setTargetPoint(new Target(ringPath.getRobotPose(Math.min(time.seconds(), ringTime))).thetaW0(ringIntakeTheta[1]));
//                    } else if (rings.size() == 3 && time.seconds() < 4.5) {
//                        robot.setTargetPoint(new Target(ringPath.getRobotPose(Math.min(time.seconds(), ringTime))).thetaW0(ringIntakeTheta[2]));
//                    }
//                } else {
                if (time.seconds() < (sweep ? 2.0 : 2.25)) {
                    robot.setTargetPoint(ringPath.getRobotPose(Math.min(time.seconds(), ringTime)));
                } else if (robot.x < 100) {
                    robot.setTargetPoint(new Target(ringPath.getRobotPose(Math.min(time.seconds(), ringTime))).thetaW0(PI/6).thetaKp(7.0).yKp(0.5));
                } else {
                    robot.setTargetPoint(ringPath.getRobotPose(Math.min(time.seconds(), ringTime)), PI/2, 0);
                }
//                }

                if (ringCase == RingCase.Four && time.seconds() > ringTime - 0.5) {
                    robot.intake.blockerHome();
                    robot.intake.sticksHalf();
                    robot.intake.reverse();
                }

                if (time.seconds() > ringTime) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 30, 0, 0),
                            new Waypoint(wobbleCor[0], wobbleCor[1], ringCase != RingCase.Four ? PI/2 : 7*PI/12, -30, -30, 0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    if (ringCase == RingCase.Four) {
                        robot.intake.stickRight(0.35);
                    }
                    robot.intake.off();

                    bounceBack = true;
                    time.reset();
                }
            }

            // Deliver first wobble goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                if (ringCase == RingCase.Four && robot.x > wobbleCor[0] - 1) {
                    robot.drivetrain.stop();
                } else {
                    robot.setTargetPoint(new Target(curPose).thetaW0(ringCase != RingCase.Four ? curPose.theta + PI : PI/2));
                }

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], PI/2)) || time.seconds() > (ringCase == RingCase.Four ? 0.25 : 2.25)) {
                    robot.wobbleArm.armDown();
                }

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], PI/2) && robot.notMoving()) || time.seconds() > (ringCase == RingCase.Four ? 0.5 : 2.75)) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > (ringCase == RingCase.Four ? 1.0 : 3.25)) {
                    reachedDeposit = false;
                    depositReachTime = 0;

                    robot.wobbleArm.armUp();

                    Waypoint[] intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -30, -50, 0, 0),
                            new Waypoint(79, 24.5, PI/2, -0.1, 60, 0, 2.5),
                            new Waypoint(87, 24.5, PI/2, -0.1, -0.1, 0, intakeWobble2Time),
                    };
                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to pick up second wobble goal
            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time);
                Pose curPose = intakeWobble2Path.getRobotPose(curTime);
                if (curTime < 2.5) {
                    robot.setTargetPoint(new Target(curPose).thetaW0(curPose.theta + PI).yKp(0.75));
                } else {
                    robot.setTargetPoint(new Target(curPose).thetaW0(PI/2).thetaKp(7.0));
                }

                if (time.seconds() > intakeWobble2Time - 0.25) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > intakeWobble2Time + 0.5) {
                    robot.shooter.flywheelHG();
                    if (ringCase == RingCase.Four) {
                        robot.intake.blockerDown();
                        robot.intake.stickRight(Constants.R_SHOOT_POS);
                    }

                    Waypoint[] goToHighShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 40, 0, 0),
                            new Waypoint(88, 61, PI/2, 30, 30, 0, goToHighShootTime),
                    };
                    goToHighShootPath = new Path(new ArrayList<>(Arrays.asList(goToHighShootWaypoints)));

                    intakeWobble2 = true;
                    time.reset();
                }
            }

            // Go to high goal shoot position
            else if (!goToHighShoot) {
                if (time.seconds() < 1) {
                    robot.setTargetPoint(robot.x, robot.y + 1, PI/2);
                } else {
                    robot.setTargetPoint(goToHighShootPath.getRobotPose(Math.min(time.seconds(), goToHighShootTime)));
                }

                if (time.seconds() > 0.5) {
                    robot.wobbleArm.clamp();
                }

                if (time.seconds() > goToHighShootTime) {
                    robot.shootYOverride = 61;
                    robot.thetaOffset = 0.07;
                    robot.highGoalShoot(ringCase == RingCase.One ? 4 : 3);

//                    robot.wobbleArm.armUp();

                    goToHighShoot = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal2) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    robot.thetaOffset = 0;

                    if (ringCase == RingCase.Four) {
                        robot.intake.blockerUp();
                        robot.intake.sticksHalf();
                        robot.intake.stickRight(0.35);
                    }

                    Waypoint[] deliverWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 50, 40, 0, 0),
                            new Waypoint(wobble2Cor[0], wobble2Cor[1], ringCase != RingCase.Four ? PI/2 : 7*PI/12, 30, 30, 0, deliverWobble2Time),
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

                if ((!reachedDeposit && robot.isAtPose(wobble2Cor[0], wobble2Cor[1], PI/2) && robot.notMoving()) || time.seconds() > 2.5) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.armDown();
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > 3.0) {

                    robot.wobbleArm.armUp();

                    Waypoint[] parkWaypoints;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
                                new Waypoint(robot.x, robot.y - 15, PI/2, -10, 0, 0, 0.75),
                                new Waypoint(85, 90, PI/3, 20, 20, 0, parkTime),
                        };
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
                                new Waypoint(90, 88, PI/2, -30, -30, 0, parkTime),
                        };
                    } else {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -50, -60, 0, 0),
                                new Waypoint(104, 88, PI/2, -30, -30, 0, parkTime),
                        };
                    }
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                if ((System.currentTimeMillis() - robot.startTime) < 30000) {
                    double curTime = Math.min(time.seconds(), parkTime);
                    Pose curPose = parkPath.getRobotPose(curTime);
                    if (ringCase == RingCase.Zero) {
                        if (curTime < 0.8) {
                            robot.setTargetPoint(curPose, PI/2, 0);
                        } else {
                            robot.setTargetPoint(curPose, 5*PI/12, 0);
                        }
                    } else if (ringCase == RingCase.Four) {
                        if (curTime < 0.5) {
                            robot.setTargetPoint(curPose, curPose.theta + PI, 0);
                        } else {
                            robot.setTargetPoint(curPose, PI/4, 0);
                        }
                    } else {
                        robot.setTargetPoint(curPose, curPose.theta + PI, 0);
                    }

                    if (ringCase == RingCase.Four && robot.y < 120) {
                        robot.intake.sticksOut();
                    }

                    if (time.seconds() > parkTime) {
                        Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");
                        robot.intake.sticksOut();

                        park = true;
                    }
                } else {
                    Robot.log("Stopping Robot");
                    robot.drivetrain.stop();
                }

                if ((System.currentTimeMillis() - robot.startTime) >= 33000) {
                    Robot.log("Breaking Loop");
                    break;
                }
            }

            else {
                robot.drivetrain.stop();
                if (robot.notMoving() || (System.currentTimeMillis() - robot.startTime) >= 30000) {
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
