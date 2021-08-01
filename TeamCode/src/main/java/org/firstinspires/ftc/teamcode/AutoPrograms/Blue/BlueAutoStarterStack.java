package org.firstinspires.ftc.teamcode.AutoPrograms.Blue;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

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
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "Blue Auto Starter Stack", preselectTeleOp = "1 Teleop", group = "Blue")
public class BlueAutoStarterStack extends LinearOpMode {

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect stack
            shoot preloaded rings into high goal
            intake 3 rings from the stack
            shoot third ring into high goal
            intake fourth ring
            shoot remaining rings from stack
            drop off wobble goal at corresponding zone
            park on line
        */

        Robot robot = new Robot(this, 30, 9, PI/2, true, false);
        robot.logger.startLogging(true, false);

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        // Segments
        boolean align = false;
        boolean goToStack = false;
        boolean shootHighGoal1 = false;
        boolean intakeStack = false;
        boolean shoot1Ring = false;
        boolean intakeStack2 = false;
        boolean shootHighGoal2 = false;
        boolean deliverWobble = false;
        boolean wobbleBack = false;
        boolean park = false;

        // Segment Times
        double goToStackTime = 1.5;
        double intakeStackTime = 3.5;
        double intakeStack2Time = 4.5;
        double deliverWobbleTime = 2.0;
        double wobbleBackTime = 1.0;
        double parkTime = 1.5;

        // Paths
        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(30, 17, PI/2, 20, 20, 0, 0),
                new Waypoint(36, 32, PI/2, 5, -10, 0, goToStackTime),
        };
        Path goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));
        Path deliverWobblePath = null;
        Path wobbleBackPath = null;
        Path parkPath = null;

        // Other variables
        boolean knockStack = false;
        int depositState = 0;
        double depositReachTime = 0;
        ArrayList<Ring> rings;
        boolean resetCalled = false;

        waitForStart();

        robot.drivetrain.updateThetaError();
        robot.wobbleArm.armUp();
        robot.intake.blockerVert();
        robot.setLockMode(Robot.TurretMode.HIGH_GOAL);

        // Determine Ring Case
        RingCase ringCase = detector.getStackPipe().getModeResult();
        Robot.log("Ring case: " + ringCase);

        // Customize Pathing Depending on Ring Case
        double[][] wobbleDelivery = {{11, 60, PI}, {11, 100, PI/2}, {13, 107, PI}};
        double[] wobbleCor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            intakeStack = true;
            shoot1Ring = true;
            intakeStack2 = true;
            goToStackWaypoints = new Waypoint[] {
                    new Waypoint(30, 17, PI/2, 30, 30, 0, 0),
                    new Waypoint(26, 60, PI/2, 5, -30, 0, goToStackTime),
            };
            goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            intakeStack2 = true;
        } else {
            wobbleCor = wobbleDelivery[2];
        }

        detector.setPipeline(Vision.Pipeline.RingLocator);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            rings = detector.getRingPipe().getRings(robot.x, robot.y, robot.theta);
            robot.ringPos = rings;

            if (!align) {
                robot.setTargetPoint(30, 17, PI/2);

                if (time.seconds() > 0.5 && !resetCalled) {
                    robot.turretReset = true;
                    resetCalled = true;
                }

                if (time.seconds() > 1.5) {
                    align = true;
                    time.reset();
                }
            }

            // Go to Starter Stack
            else if (!goToStack) {
                double curTime = Math.min(time.seconds(), goToStackTime);
                Pose curPose = goToStackPath.getRobotPose(curTime);
                robot.setTargetPoint(new Target(curPose).thetaKp(3.5));

                if (ringCase == RingCase.Four || time.seconds() > goToStackTime + 3.5) {
                    robot.shooter.setFlywheelVelocity(ringCase == RingCase.Zero ? 1570 : 1630);
                }

                if (((ringCase == RingCase.Four && time.seconds() > goToStackTime) || (ringCase != RingCase.Four && time.seconds() > goToStackTime + 5))
                        && robot.isAtPose(36, ringCase == RingCase.Zero ? 60 : 32, PI/2, 20, 20, PI/35) && robot.notMoving()) {
                    robot.shootYOverride = ringCase == RingCase.Zero ? 60 : 32;
                    robot.highGoalShoot(ringCase == RingCase.One ? 2 : 4, ringCase == RingCase.Zero ? 1570 : 1630);

                    if (ringCase == RingCase.Zero) {
                        robot.intake.bumpersOut();
                    } else {
                        robot.intake.blockerVert();
                    }

                    goToStack = true;
                    time.reset();
                }
            }

            // Shoot Rings in High Goal
            else if (!shootHighGoal1) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (ringCase != RingCase.Zero) {
                        robot.shooter.setFlywheelVelocity(1630);
                    }

                    shootHighGoal1 = true;
                    time.reset();
                }
            }

            // Intake Rings from Starter Stack
            else if (!intakeStack) {
                if (ringCase == RingCase.Four) {
                    if (time.seconds() > intakeStackTime - 0.5) {
                        robot.intake.setPower(-0.35);
                    } else if (knockStack) {
                        robot.intake.on();
                        robot.setTargetPoint(new Target(36, Math.min(39 + 2.5 * time.seconds(), 46), PI/2).thetaKp(3.5));
                    } else if (robot.isAtPose(36, 45, PI/2, 0.5, 0.5, PI/35) && robot.notMoving()) {
                        robot.drivetrain.stop();
                        knockStack = true;
                    } else {
                        robot.setTargetPoint(new Target(36, 45, PI/2).thetaKp(3.5));
                    }
                } else {
                    robot.intake.on();
                    robot.setTargetPoint(36, 60, PI/2);
                }

                if (time.seconds() > intakeStackTime) {
                    robot.shootYOverride = robot.y;
                    robot.highGoalShoot(ringCase == RingCase.One ? 4 : 2, 1630);

                    intakeStack = true;
                    time.reset();
                }
            }

            // Shoot 1 Ring in High Goal
            else if (!shoot1Ring) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (ringCase == RingCase.Four) {
                        robot.intake.on();
                    } else {
                        robot.intake.bumpersOut();
                    }
                    shoot1Ring = true;
                    time.reset();
                }
            }

            // Intake Fourth Ring from Stack
            else if (!intakeStack2) {
                if (time.seconds() < 4.5) {
                    robot.setTargetPoint(new Target(36, Math.min(46 + 3 * time.seconds(), 63), PI/2).thetaKp(3.5));
                } else {
                    robot.setTargetPoint(36, 63, PI/2);
                }

                if (time.seconds() > intakeStack2Time - 1) {
                    robot.shooter.setFlywheelVelocity(1550);
                }

                if (time.seconds() > intakeStack2Time) {
                    robot.highGoalShoot(3, 1550);
                    robot.intake.bumpersOut();

                    intakeStack2 = true;
                    time.reset();
                }
            }

            // Shoot Rings in High Goal
            else if (!shootHighGoal2) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                            new Waypoint(wobbleCor[0], wobbleCor[1], wobbleCor[2], 5, 0, 0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    robot.intake.setBlockerPos(0.45);

                    shootHighGoal2 = true;
                    time.reset();
                }
            }

            // Deliver Wobble Goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                if (depositState == 0 && (robot.isAtPose(wobbleCor[0], wobbleCor[1], wobbleCor[2]) && robot.notMoving() || time.seconds() > deliverWobbleTime + 1)) {
                    robot.wobbleArm.armDown();
                    depositReachTime = time.seconds();
                    depositState = 1;
                }

                if (depositState == 1 && (time.seconds() > depositReachTime + 1 || time.seconds() > deliverWobbleTime + 2.5)) {
                    depositState = 2;
                    depositReachTime = time.seconds();
                    robot.wobbleArm.unClamp();
                }

                if (depositState == 2 && (time.seconds() > depositReachTime + 0.75 || time.seconds() > deliverWobbleTime + 4)) {
                    robot.wobbleArm.armInside();
                    robot.wobbleArm.clawIn();
                    robot.intake.bumpersHome();
                    robot.moveWobbleOut = 1;
                    depositReachTime = time.seconds();
                    depositState = 3;
                }

                if (depositState == 3 && (time.seconds() > depositReachTime + 1.5 || time.seconds() > deliverWobbleTime + 6)) {

                    robot.intake.blockerHome();
                    robot.moveWobbleOut = 0;

                    Waypoint[] wobbleBackWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta,  -20, -10, 0, 0),
                            new Waypoint(robot.x + 10, robot.y - 2, robot.theta, -10, -10, 0, wobbleBackTime),
                    };
                    wobbleBackPath = new Path(new ArrayList<>(Arrays.asList(wobbleBackWaypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            else if (!wobbleBack) {
                double curTime = Math.min(time.seconds(), wobbleBackTime);
                Pose curPose = wobbleBackPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.x, curPose.y, curPose.theta + PI);

                if (time.seconds() > wobbleBackTime || ringCase == RingCase.One) {
                    Waypoint[] parkWaypoints;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                                new Waypoint(robot.x + 10, robot.y, PI, 5, 5, 0, 0.75),
                                new Waypoint(37, 82, PI/2, 20, 10, 0, parkTime),
                        };
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -10, -5, 0, 0),
                                new Waypoint(18, 80, PI/2, -10, -5, 0, parkTime),
                        };
                    } else {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -50, -40, 0, 0),
                                new Waypoint(24, 80, PI/2, -30, -10, 0, parkTime),
                        };
                    }
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    wobbleBack = true;
                    time.reset();
                }
            }

            // Park
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.x, curPose.y, curPose.theta + (ringCase != RingCase.Zero ? PI : 0));

                if (curTime > parkTime) {
                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;
                }
            }

            else {
                robot.drivetrain.stop();
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
