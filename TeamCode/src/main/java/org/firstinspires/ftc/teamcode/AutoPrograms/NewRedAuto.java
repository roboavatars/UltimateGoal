package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.RingLocator;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeightDetector;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeightPipeline.RingCase;
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

        RingLocator locator = null; // cannot run two pipelines at same time
        ArrayList<double[]> ringPos = new ArrayList<>(3);
        robot.ringPos = ringPos;

        // Segments
        boolean goToStack = false;
        boolean shootHighGoal = false;
        boolean intakeStack = false;
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
        double goToStackTime = 1.0;
        double shootHighGoal1Time = 1.5;
        double intakeStackTime = 1.5;
        double shootHighGoal1RingTime = 2.5;
        double intakeStack2Time = 3.0;
        double goToPowerShootTime = 1.0;
        double shootPowerShotsTime = 1.5;
        double bounceBackTime = 6.0;
        double deliverWobbleTime = 1.0;
        double intakeWobble2Time = 3.5;
        double goToHighShootTime = 0.5;
        double shootHighGoal2Time = 1.5;
        double deliverWobble2Time = 1.75;
        double parkTime = 2.0;

        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(114, 9, PI/2, 40, 50, 0, 0),
                new Waypoint(107, 38, PI/2, 40, 30, 0, goToStackTime),
        };
        Path goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));

        // Paths and Theta splines
        Path goToPowerShootPath = null;
        Path bounceBackPath = null;
        Path deliverWobblePath = null;
        Path intakeWobble2Path = null;
        Path goToHighShootPath = null;
        Path deliverWobble2Path = null;
        Path parkPath = null;
        Spline parkThetaSpline = null;

        // Other variables
        boolean shotRing = false;
        boolean psFinish = false;
        double psFinishTime = 0;
        boolean reachedWgPickup = false;
        double reachedTime = 0;

        waitForStart();

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getModeResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{121, 82}, {96, 103}, {122, 130}};
        double[][] wobble2Delivery = {{119, 74}, {96, 92}, {123, 127}};
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
        robot.wobbleArm.setArmPosition(-120);
        robot.wobbleArm.clamp();

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            // Go to starting stack
            if (!goToStack) {
                robot.setTargetPoint(goToStackPath.getRobotPose(Math.min(time.seconds(), goToStackTime)), 0.8);

                robot.shooter.flywheelHG();

                if (time.seconds() > goToStackTime) {
                    robot.wobbleArm.setArmPosition(-300);

                    robot.shootYOverride = robot.y;
                    robot.highGoalShoot();

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
                if (robot.y < 50) {
//                    double input = Math.min(55, 40 + 6 * time.seconds() + 1.5 * Math.sin(8 * time.seconds()));
//                    robot.setTargetPoint(108, input, PI/2, 0.8, 0.6, 6);
                    robot.setTargetPoint(108, 50, PI/2);
                } else if (!shotRing && robot.y > 50) {
                    robot.shootYOverride = 50;
                    robot.highGoalShoot(1);
                    shotRing = true;
                } else if (shotRing && !robot.preShoot && !robot.shoot && robot.numRings == 0 && robot.y > 50) {
                    robot.setTargetPoint(108, 55, PI/2);
                }

                if (time.seconds() > intakeStack2Time) {

                    robot.shooter.flywheelPS();

                    locator = new RingLocator(this);
                    locator.start();

                    Waypoint[] goToPowerShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 30, 20, 0, 0),
                            new Waypoint(87, 63, PI/2, 30, 30, 0, goToPowerShootTime),
                    };
                    goToPowerShootPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShootWaypoints)));

                    intakeStack = true;
                    time.reset();
                }
            }

            // Go to powershot shoot position
            else if (!goToPowerShoot) {
                robot.setTargetPoint(goToPowerShootPath.getRobotPose(Math.min(time.seconds(), goToPowerShootTime)));

                if (time.seconds() > goToPowerShootTime) {
                    robot.powerShotShoot();

                    goToPowerShoot = true;
                    time.reset();
                }
            }

            // Time block to shoot powershot
            else if (!shootPowerShots) {
                double curTime = time.seconds();
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!psFinish) {
                        psFinish = true;
                        psFinishTime = curTime;
                    } else if (curTime > psFinishTime + 0.25) {

                        robot.intake.on();

//                        ringPos.add(locator.getAbsRingPos(robot.x, robot.y, robot.theta));
                        ringPos.add(new double[] {86, 130});
                        ringPos.add(new double[] {94, 110});
                        ringPos.add(new double[] {110, 120});
                        locator.stop();

                        Waypoint[] bounceBackWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 50, 60, 0, 0),

                                new Waypoint(ringPos.get(0)[0], ringPos.get(0)[1] - 15, PI/2, 30, 40, 0, 1.5),
                                new Waypoint(ringPos.get(0)[0] + 3, ringPos.get(0)[1] - 5, PI/2, -30, -10, 0, 3.0),

                                new Waypoint(ringPos.get(1)[0], ringPos.get(1)[1] - 15, PI/2, 30, 30, 0, 4.0),
                                new Waypoint(ringPos.get(1)[0] + 3, ringPos.get(1)[1] - 5, PI/2, -30, -10, 0, 5.0),

                                new Waypoint(ringPos.get(2)[0], ringPos.get(2)[1], PI/2, 30, 40, 0, bounceBackTime),
                        };
                        bounceBackPath = new Path(new ArrayList<>(Arrays.asList(bounceBackWaypoints)));

                        shootPowerShots = true;
                        time.reset();
                    }
                }
            }

            // Collect bounce backed rings
            else if (!bounceBack) {
                robot.setTargetPoint(bounceBackPath.getRobotPose(Math.min(time.seconds(), bounceBackTime)), PI/2, 0, 2);

                if (time.seconds() > bounceBackTime) {

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

                if (time.seconds() > deliverWobbleTime + 0.75) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (time.seconds() > deliverWobbleTime) {
                    robot.wobbleArm.unClamp();
                } else if (time.seconds() > deliverWobbleTime - 1) {
                    robot.wobbleArm.setArmPosition(-600);
                }

                if (time.seconds() > deliverWobbleTime + 1) {

                    robot.wobbleArm.clamp();
                    robot.intake.off();

                    Waypoint[] intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 40, 0, 0),
                            new Waypoint(100, 36.5, 5*PI/12, 0, 40, 0, intakeWobble2Time),
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
                robot.setTargetPoint(curPose, 5*PI/12, 0);

                if (!reachedWgPickup && robot.isAtPose(100, 36.5, 5*PI/12, 0.5, 0.5, PI/12)) {
                    reachedWgPickup = true;
                    reachedTime = curTime;
                }

                if (reachedWgPickup && time.seconds() > reachedTime + 1) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (reachedWgPickup && time.seconds() > reachedTime + 0.5) {
                    robot.wobbleArm.clamp();
                } else if (time.seconds() > intakeWobble2Time - 1.5) {
                    robot.wobbleArm.unClamp();
                } else if (time.seconds() > intakeWobble2Time - 2) {
                    robot.wobbleArm.down();
                }

                if (reachedWgPickup && time.seconds() > reachedTime + 1.5) {

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
                    robot.highGoalShoot();

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
                robot.setTargetPoint(deliverWobble2Path.getRobotPose(Math.min(time.seconds(), deliverWobble2Time)));

                if (time.seconds() > deliverWobble2Time + 0.75) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (time.seconds() > deliverWobble2Time - 0.25) {
                    robot.wobbleArm.unClamp();
                } else if (time.seconds() > deliverWobble2Time - 1) {
                    robot.wobbleArm.setArmPosition(-600);
                }

                if (time.seconds() > deliverWobble2Time + 1) {

                    robot.wobbleArm.clamp();

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
                    parkThetaSpline = new Spline(robot.theta, 0, 0, PI/2, 0, 0, parkTime);
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble2 = true;
                    time.reset();
                }
            }

            // Park on line
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                robot.setTargetPoint(parkPath.getRobotPose(curTime), parkThetaSpline.position(curTime), parkThetaSpline.velocity(curTime));

                if (time.seconds() > parkTime) {
                    robot.intake.sticksOut();
                    robot.wobbleArm.down();

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
