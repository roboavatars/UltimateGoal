package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Dashboard;
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

@Autonomous(name = "1 NewRedAuto")
public class NewRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect stack
            move to line to shoot the 3 preloaded rings at powershots
            intake 3 rings from stack
            shoot rings into high goal
            detect rings that bounced back from powershot
            intake last rings
            collect powershot bounce backs
            drop off wobble goal at corresponding zone
            go back to pick up second wobble
            shoot bounce back rings into high goal
            drop off second wobble at corresponding zone
            park on line
        */

        Robot robot = new Robot(this, 90, 9, PI/2, true);
        robot.logger.startLogging();
        robot.intake.sticksHome();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

        RingLocator locator = null; //new RingLocator(this);
        ArrayList<double[]> rings = new ArrayList<>(3);
//        rings.add(null);
//        rings.add(null);
//        rings.add(null);

        // Segments
        boolean startLine = false;
        boolean shootPowerShots = false;
        boolean goToStack = false;
        boolean intakeStack = false;
        boolean shootHighGoal = false;
        boolean bounceBack = false;
        boolean deliverWobble = false;
        boolean intakeWobble2 = false;
        boolean goToHighShoot = false;
        boolean shootHighGoal2 = false;
        boolean bounceDeliverWobble2 = false;
        boolean park = false;

        // Segment Times
        double startLineTime = 1.5;
        double shootPowerShotsTime = 2.0;
        double goToStackTime = 2.0;
        double intakeStackTime = 2.0;
        double shootHighGoal1Time = 2.0;
        double bounceBackTime = 3.5;
        double deliverWobbleTime = 1.5;
        double intakeWobble2Time = 4.0;
        double goToHighShootTime = 0.75;
        double shootHighGoal2Time = 2.0;
        double bounceDeliverWobble2Time = 3.0;
        double parkTime = 1.5;

        Waypoint[] startLineWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 40, 50, 0, 0),
                new Waypoint(87, 63, PI/2, 10, -30, 0, startLineTime),
        };
        Path startLinePath = new Path(new ArrayList<>(Arrays.asList(startLineWaypoints)));

        // Paths and Theta splines
        Path goToStackPath = null;
        Spline goToStackThetaSpline = null;
        Path bounceBackPath = null;
        Path deliverWobblePath = null;
        Path intakeWobble2Path = null;
        Spline intakeWobble2ThetaSpline = null;
        Spline intakeWobble2ThetaSpline2 = null;
        Path goToHighShootPath = null;
        Path bounceDeliverWobble2Path = null;
        Path parkPath = null;
        Spline parkThetaSpline = null;

        // Other variables
        boolean psFinish = false;
        double psFinishTime = 0;
        boolean startOscillation = false;
        boolean reached = false;
        double reachedTime = 0;

        waitForStart();

        // Wobble coordinates based on ring case
        RingCase ringCase = detector.getModeResult();
        Robot.log("Ring case: " + ringCase);

        double[][] wobbleDelivery = {{121, 82}, {96, 103}, {125, 133}};
        double[][] wobble2Delivery = {{119, 74}, {96, 92}, {123, 127}};
        double[] wobbleCor;
        double[] wobble2Cor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            wobble2Cor = wobble2Delivery[0];
            intakeStack = true;
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
        robot.wobbleArm.clampWobble();

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            for (double[] ring : rings) {
                Dashboard.drawRing(ring[0], ring[1]);
            }

            // Go to shooting line to shoot powershots
            if (!startLine) {
                double curTime = Math.min(time.seconds(), startLineTime);
                robot.setTargetPoint(startLinePath.getRobotPose(curTime));

                if (time.seconds() > startLineTime - 1.25) {
                    robot.wobbleArm.setArmPosition(-300);
                }

                if (time.seconds() > startLineTime) {
                    robot.powerShotShoot();

                    startLine = true;
                    time.reset();
                }
            }

            // Time block to shoot powershots
            else if (!shootPowerShots) {
                double curTime = time.seconds();
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (!psFinish) {
                        psFinish = true;
                        psFinishTime = curTime;
                    } else if (curTime > psFinishTime + 0.2) {

                        robot.intake.intakeOn();

                        Waypoint[] goToStackWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 30, 30, 0, 0),
                                new Waypoint(98, 48, 0, 20, 20, 0, goToStackTime),
                        };
                        goToStackThetaSpline = new Spline(robot.theta, -3, 0, 0, 0, 0, goToStackTime);
                        goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));

                        shootPowerShots = true;
                        time.reset();
                    }
                }
            }

            else if (!goToStack) {
                double curTime = Math.min(time.seconds(), goToStackTime);
                Pose curPose = goToStackPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.getX(), curPose.getY(), goToStackThetaSpline.position(curTime), 0.3, 0.3, 3.5);

                if (time.seconds() > goToStackTime) {
                    goToStack = true;
                    time.reset();
                }
            }

            // Intake start stack
            else if (!intakeStack) {

//                double input = Math.min(110, 100 + 6 * time.seconds() + 1.5 * Math.sin(8 * time.seconds()));
//                robot.setTargetPoint(input, 48, 0, 0.8, 0.6, 6);
                robot.setTargetPoint(110, 48, 0);

                if (time.seconds() > intakeStackTime) {
                    robot.highGoalShoot();

                    locator = new RingLocator(this);
                    locator.start();

                    intakeStack = true;
                    time.reset();
                }
            }

            // Time block to shoot into high goal
            else if (!shootHighGoal) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {

                    robot.intake.intakeOn();

                    rings.add(locator.getAbsRingPos(robot.x, robot.y, robot.theta));
                    rings.add(new double[] {86, 130});
//                    rings.add(new double[] {94, 110});
                    rings.add(new double[] {110, 120});
                    locator.stop();

                    Waypoint[] bounceBackWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 50, 60, 0, 0),

                            new Waypoint(rings.get(2)[0], rings.get(2)[1], PI/2, 30, -30, 0, 1.5),
                            new Waypoint(rings.get(2)[0] - 2, rings.get(2)[1] - 10, PI/2, -40, 30, 0, 2.25),

                            new Waypoint(rings.get(1)[0], rings.get(1)[1], PI/2, 40, 40, 0, bounceBackTime),
                    };
                    bounceBackPath = new Path(new ArrayList<>(Arrays.asList(bounceBackWaypoints)));

                    shootHighGoal = true;
                    time.reset();
                }
            }

            // Collect bounce backed rings
            else if (!bounceBack) {
                double curTime = Math.min(time.seconds(), bounceBackTime);
                robot.setTargetPoint(bounceBackPath.getRobotPose(curTime), PI/2, 0);

                if (time.seconds() > bounceBackTime) {

                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
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

                if (time.seconds() > deliverWobbleTime + 1) {
                    robot.wobbleArm.setArmPosition(-300);
                } else if (time.seconds() > deliverWobbleTime + 0.75) {
                    robot.wobbleArm.unClampWobble();
                } else if (time.seconds() > deliverWobbleTime + 0.25) {
                    robot.wobbleArm.armDown();
                }

                if (time.seconds() > deliverWobbleTime + 1.25) {

                    robot.intake.intakeOff();

                    Waypoint[] intakeWobble2Waypoints;
//                    if (ringCase == RingCase.Zero) {
//                        intakeWobble2Waypoints = new Waypoint[] {
//                                new Waypoint(robot.x, robot.y, robot.theta, 30, 50, 0, 0),
//                                new Waypoint(wobbleCor[0] - 5, wobbleCor[1] - 5, robot.theta, -20, -40, 0, 0.5),
//                                new Waypoint(127, 63, PI/2, -20, -5, 0, 1),
//                                new Waypoint(124, 36.5, 5*PI/12, 0, 30, 0, intakeWobble2Time),
//                        };
//                        intakeWobble2ThetaSpline = new Spline(robot.theta, -8, 0, 5*PI/12, 0, 0, intakeWobble2Time);
//                    } else if (ringCase == RingCase.One) {
//                        intakeWobble2Waypoints = new Waypoint[] {
//                                new Waypoint(robot.x, robot.y, robot.theta, 30, 50, 0, 0),
//                                new Waypoint(wobbleCor[0] - 5, wobbleCor[1] - 9, robot.theta, -20, -40, 0, 0.5),
//                                new Waypoint(127, 66, PI/2, -30, -5, 0, 1.5),
//                                new Waypoint(124, 36.5, 5*PI/12, 0, 30, 0, intakeWobble2Time),
//                        };
//                        intakeWobble2ThetaSpline = new Spline(robot.theta, -8, 0, 5*PI/12, 0, 0, intakeWobble2Time);
//                    } else {
                    intakeWobble2Waypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                            new Waypoint(wobbleCor[0] - 5, wobbleCor[1] - 9, robot.theta, -30, -30, 0, 1),
                            new Waypoint(128, 53, 3*PI/2, 40, 30, 0, 2),
                            new Waypoint(124, 36.5, 5*PI/12, 0, 40, 0, intakeWobble2Time),
                    };
                        intakeWobble2ThetaSpline = new Spline(robot.theta, 7, 0, 3*PI/2, 0, 0, intakeWobble2Time-2);
                        intakeWobble2ThetaSpline2 = new Spline(robot.theta, -6, 0, 5*PI/12, 0, 0, intakeWobble2Time);
//                    }
                    intakeWobble2Path = new Path(new ArrayList<>(Arrays.asList(intakeWobble2Waypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to pick up second wobble goal
            else if (!intakeWobble2) {
                double curTime = Math.min(time.seconds(), intakeWobble2Time);
                Pose curPose = intakeWobble2Path.getRobotPose(curTime);
                if (robot.y > 115) {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), intakeWobble2ThetaSpline.position(curTime));
                } else if (robot.y > 53 && robot.y < 115) {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), 3*PI/2);
                } else if (robot.y < 53) {
                    robot.setTargetPoint(curPose.getX(), curPose.getY(), intakeWobble2ThetaSpline2.position(curTime));
                }

                if (robot.y < 72) {
                    robot.intake.intakeOn();
                }

                if (!reached && robot.isAtPose(124, 36.5, 5*PI/12, 0.5, 0.5, PI/35)) {
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

                    robot.intake.intakeOff();

                    Waypoint[] goToHighShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 30, 30, 0, 0),
                            new Waypoint(108, 64, PI/2, 30, 20, 0, goToHighShootTime),
                    };
                    goToHighShootPath = new Path(new ArrayList<>(Arrays.asList(goToHighShootWaypoints)));

                    intakeWobble2 = true;
                    time.reset();
                }
            }

//            // Go to high goal shoot spot
//            else if (!goToHighShoot) {
//                double curTime = Math.min(time.seconds(), startLineTime);
//                robot.setTargetPoint(goToHighShootPath.getRobotPose(curTime));
//
//                if (time.seconds() > goToHighShootTime) {
//                    robot.highGoalShoot();
//
//                    goToHighShoot = true;
//                    time.reset();
//                }
//            }
//
//            // Time block to shoot into high goal
//            else if (!shootHighGoal2) {
//                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
//
//                    robot.intake.intakeOn();
//
//                    Waypoint[] bounceDeliverWobble2Waypoints = new Waypoint[] {
//                            new Waypoint(robot.x, robot.y, robot.theta, 50, 40, 0, 0),
//                            new Waypoint(rings.get(0)[0], rings.get(0)[1] - 2, 2*PI/3, 20, 30, 0, 1.75),
//                            new Waypoint(wobble2Cor[0], wobble2Cor[1], PI, 30, 20, 0, bounceDeliverWobble2Time),
//                    };
//                    bounceDeliverWobble2Path = new Path(new ArrayList<>(Arrays.asList(bounceDeliverWobble2Waypoints)));
//
//                    shootHighGoal2 = true;
//                    time.reset();
//                }
//            }
//
//            // Deliver second wobble goal
//            else if (!bounceDeliverWobble2) {
//                double curTime = Math.min(time.seconds(), bounceDeliverWobble2Time);
//                if (robot.y < 120) {
//                    robot.setTargetPoint(bounceDeliverWobble2Path.getRobotPose(curTime));
//                } else {
//                    robot.setTargetPoint(bounceDeliverWobble2Path.getRobotPose(curTime), PI, 0);
//                }
//
//                if (time.seconds() > bounceDeliverWobble2Time + 1) {
//                    robot.wobbleArm.setArmPosition(-300);
//                } else if (time.seconds() > bounceDeliverWobble2Time + 0.75) {
//                    robot.wobbleArm.unClampWobble();
//                } else if (time.seconds() > bounceDeliverWobble2Time + 0.25) {
//                    robot.wobbleArm.armDown();
//                }
//
//                if (time.seconds() > bounceDeliverWobble2Time + 1.25) {
//
//                    robot.wobbleArm.clampWobble();
//                    robot.intake.intakeOff();
//
//                    Waypoint[] parkWaypoints;
//                    if (ringCase == RingCase.Zero) {
//                        parkWaypoints = new Waypoint[] {
//                                new Waypoint(robot.x, robot.y, robot.theta, -20, -20, 0, 0),
//                                new Waypoint(112, 65, robot.theta, -20, -20, 0, 1),
//                                new Waypoint(94, 80, PI/2, 30, 20, 0, parkTime),
//                        };
//                    } else if (ringCase == RingCase.One) {
//                        parkWaypoints = new Waypoint[] {
//                                new Waypoint(robot.x, robot.y, robot.theta, -30, -30, 0, 0),
//                                new Waypoint(76, 80, PI/2, 30, 20, 0, parkTime),
//                        };
//                    } else {
//                        parkWaypoints = new Waypoint[] {
//                                new Waypoint(robot.x, robot.y, robot.theta, 40, 50, 0, 0),
//                                new Waypoint(98, 85, PI/2, 30, -30, 0, parkTime),
//                        };
//                    }
//                    parkThetaSpline = new Spline(robot.theta, 0, 0, PI/2, 0, 0, parkTime);
//                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));
//
//                    bounceDeliverWobble2 = true;
//                    time.reset();
//                }
//            }
//
//            // Park on line
//            else if (!park) {
//                double curTime = Math.min(time.seconds(), parkTime);
//                robot.setTargetPoint(parkPath.getRobotPose(curTime), parkThetaSpline.position(curTime), parkThetaSpline.velocity(curTime));
//
//                if (time.seconds() > parkTime) {
//                    robot.intake.sticksOut();
//                    robot.wobbleArm.armDown();
//
//                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");
//
//                    park = true;
//                }
//            }

            else {
                robot.drivetrain.setControls(0, 0, 0);
            }

            robot.update();
        }

        robot.shooter.feedHome();
        robot.stop();
    }
}
