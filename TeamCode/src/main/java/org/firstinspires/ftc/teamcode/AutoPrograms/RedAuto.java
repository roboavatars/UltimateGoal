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
        double intakeStackTime = 2.0;
        double shoot1RingTime = 0.75;
        double intakeStack2Time = 1.0;
        double shootPowerShotsTime = 3.0;
        double deliverWobbleTime = 1.75;
        double ringTime = 0;
        double intakeWobble2Time = 3.5;
        double goToHighShootTime = 1.0;
        double shootHighGoal2Time = 1.5;
        double deliverWobble2Time = 1.5;
        double parkTime = 1.5;

        // Paths
        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(114, 9, PI/2, 40, 50, 0, 0),
                new Waypoint(110, 36, PI/2, 40, 30, 0, goToStackTime),
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

        double[][] wobbleDelivery = {{123, 92}, {97, 115}, {123, 132}};
        double[][] wobble2Delivery = {{119, 85}, {92, 106}, {116, 128}};
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
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            wobble2Cor = wobble2Delivery[1];
        } else {
            wobbleCor = wobbleDelivery[2];
            wobble2Cor = wobble2Delivery[2];
            intakeStack2Time = 2.0;
        }

        detector.setPipeline(Vision.Pipeline.RingLocator);
        robot.intake.stickLeft(Constants.L_SWEEP_POS);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            rings = detector.getRingPipe().getRings(robot.x, robot.y, robot.theta);
            robot.ringPos = rings;

            // Go to starting stack
            if (!goToStack) {
                robot.setTargetPoint(goToStackPath.getRobotPose(Math.min(time.seconds(), goToStackTime)), PI/2, 0);

                if (time.seconds() > 0.5) {
                    if (ringCase == RingCase.Four) {
                        robot.intake.knockStack();
                    } else {
                        robot.intake.blockerDown();
                    }
                }

                if (ringCase != RingCase.Zero) {
                    robot.shooter.flywheelHG();
                } else {
                    robot.shooter.flywheelPS();

                    if (time.seconds() > 1) {
                        robot.intake.stickLeft(0.75);
                    }
                }

                if (time.seconds() > goToStackTime) {

                    if (ringCase != RingCase.Zero) {
                        robot.shootYOverride = 36;
                        robot.thetaOffset = 0.03;
                        Constants.HIGH_GOAL_VELOCITY = 1770;
                        robot.highGoalShoot(ringCase == RingCase.Four ? 3 : 1);
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
                    }

                    shootHighGoal = true;
                    time.reset();
                }
            }

            // Intake start stack
            else if (!intakeStack) {
                robot.setTargetPoint(110, 43, PI/2);

                if (time.seconds() > 0.75) {
                    robot.intake.setPower(0.8);
                    robot.intake.blockerDown();
                }

                if (time.seconds() > intakeStackTime) {
                    if (ringCase == RingCase.Four) {
                        robot.shootYOverride = robot.y;
                        robot.highGoalShoot(1);
                    }

                    if (ringCase == RingCase.One) {
                        Waypoint[] intakeStack2Waypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                                new Waypoint(111, 63, PI/2, 20, 20, 0, intakeStack2Time),
                        };
                        intakeStack2Path = new Path(new ArrayList<>(Arrays.asList(intakeStack2Waypoints)));
                    }

                    robot.intake.stickLeft(Constants.L_OUT_POS);

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

                    if (ringCase == RingCase.Four) {
                        double input = Math.min(63, 42 + 4 * time.seconds() + 2.5 * Math.sin(8 * time.seconds()));
                        robot.setTargetPoint(108, input, PI/2);
                    } else if (ringCase == RingCase.One) {
                        robot.setTargetPoint(intakeStack2Path.getRobotPose(Math.min(time.seconds(), intakeStack2Time)));
                    }

                    if (time.seconds() > intakeStack2Time - 1) {
                        robot.shooter.flywheelPS();
                    } else if (time.seconds() > intakeStack2Time - 0.25) {
                        robot.intake.blockerUp();
                        robot.intake.reverse();
                    }

                    if (time.seconds() > intakeStack2Time) {
                        robot.thetaOffset = 0;
                        Constants.HIGH_GOAL_VELOCITY = 1850;
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
                        robot.drivetrain.setControls(0, 0, 0);
                    } else if (time.seconds() > psFinishTime + 0.25) {
                        robot.intake.on();

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

                                    // 3rd ring?
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

                if (time.seconds() > ringTime - 1.5) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > ringTime) {

                    robot.wobbleArm.armUp();

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
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.setTargetPoint(new Target(curPose).thetaW0(curPose.theta + PI).xyKd(0).thetaKd(0));

                if ((ringCase == RingCase.Four && sweep)
                        || (!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], PI/2)) || time.seconds() > 2.25) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.armDown();
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > 2.75) {
                    reachedDeposit = false;
                    depositReachTime = 0;

                    robot.intake.off();
                    robot.wobbleArm.armUp();

                    Waypoint[] intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -30, -50, 0, 0),
                            new Waypoint(82, 24, PI/2, -0.1, 30, 0, 2.5),
                            new Waypoint(85.5, 24, PI/2, -0.1, -0.1, 0, intakeWobble2Time),
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
                    robot.setTargetPoint(curPose, curPose.theta + PI, 0);
                } else {
                    robot.setTargetPoint(curPose, PI/2, 0);
                }

                if (time.seconds() > intakeWobble2Time - 0.5) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > intakeWobble2Time + 0.5) {
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

                if (time.seconds() > 0.35) {
                    robot.wobbleArm.armUp();
                } else if (time.seconds() > 0.15) {
                    robot.wobbleArm.clamp();
                }

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

                if ((!reachedDeposit && robot.isAtPose(wobble2Cor[0], wobble2Cor[1], PI/2)) || time.seconds() > 2.25) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.armDown();
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > 2.75) {

                    robot.wobbleArm.armUp();

                    Waypoint[] parkWaypoints;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
                                new Waypoint(robot.x, robot.y - 4, PI/2, -10, 0, 0, 0.5),
                                new Waypoint(89, 84, PI/2, 20, 20, 0, parkTime),
                        };
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
                                new Waypoint(90, 84, PI/2, -30, -30, 0, parkTime),
                        };
                    } else {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -50, -100, 0, 0),
                                new Waypoint(104, 86, PI/2, -30, -30, 0, parkTime),
                        };
                    }
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                if (ringCase == RingCase.Zero) {
                    robot.setTargetPoint(parkPath.getRobotPose(curTime));
                } else if (ringCase == RingCase.One) {
                    robot.setTargetPoint(curPose, curPose.theta + PI, 0);
                } else {
                    if (curTime < 1) {
                        robot.setTargetPoint(curPose, 5*PI/4, 0);
                    } else {
                        robot.intake.sticksCollect();
                    }
                }

                if (time.seconds() > parkTime) {
                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");
                    robot.intake.sticksCollect();

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
