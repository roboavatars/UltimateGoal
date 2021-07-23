package org.firstinspires.ftc.teamcode.AutoPrograms.Blue;

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

        Robot robot = new Robot(this, 114, 9, PI/2, true);
        robot.logger.startLogging(true);

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        // Segments
        boolean goToStack = false;
        boolean shootHighGoal1 = false;
        boolean intakeStack = false;
        boolean shoot1Ring = false;
        boolean intakeStack2 = false;
        boolean shootHighGoal2 = false;
        boolean deliverWobble = false;
        boolean park = false;

        // Segment Times
        double goToStackTime = 1.0;
        double intakeStackTime = 2.0;
        double intakeStack2Time = 2.25;
        double deliverWobbleTime = 1.75;
        double parkTime = 1.5;

        // Paths
        Waypoint[] goToStackWaypoints = new Waypoint[] {
                new Waypoint(114, 9, PI/2, 30, 30, 0, 0),
                new Waypoint(110, 32, PI/2, 5, -30, 0, goToStackTime),
        };
        Path goToStackPath = new Path(new ArrayList<>(Arrays.asList(goToStackWaypoints)));
        Path deliverWobblePath = null;
        Path parkPath = null;

        // Other variables
        final int highGoalVelocity = Constants.HIGH_GOAL_VELOCITY;
        boolean knockStack = false;
        boolean reachedDeposit = false;
        double depositReachTime = 0;
        ArrayList<Ring> rings;

        waitForStart();

        robot.drivetrain.updateThetaError();

        // Determine Ring Case
        RingCase ringCase = detector.getStackPipe().getModeResult();
        Robot.log("Ring case: " + ringCase);

        // Customize Pathing Depending on Ring Case
        double[][] wobbleDelivery = {{130, 70, 5*PI/6}, {108, 93, 5*PI/6}, {126, 119, 2*PI/3}};
        double[] wobbleCor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
            intakeStack = true;
            shoot1Ring = true;
            intakeStack2 = true;
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

            // Go to Starter Stack
            if (!goToStack) {
                double curTime = Math.min(time.seconds(), goToStackTime);
                Pose curPose = goToStackPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                if (ringCase != RingCase.Zero && time.seconds() > 0.4) {
                    robot.intake.blockerDown();
                }

                robot.shooter.flywheelHG();

                if (time.seconds() > goToStackTime) {
                    robot.shootYOverride = 32;
                    robot.highGoalShoot(4, true);

                    goToStack = true;
                    time.reset();
                }
            }

            // Shoot Rings in High Goal
            else if (!shootHighGoal1) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    if (ringCase != RingCase.Zero) {
                        robot.shooter.flywheelHG();
                    }

                    shootHighGoal1 = true;
                    time.reset();
                }
            }

            // Intake Rings from Starter Stack
            else if (!intakeStack) {
                if (ringCase == RingCase.Four) {
                    if (time.seconds() > intakeStackTime - 0.25) {
                        robot.intake.setPower(-0.5);
                    } else if (knockStack) {
                        robot.intake.on();
                        robot.setTargetPoint(new Target(110, Math.min(36 + 4.5 * time.seconds(), 41), PI/2).thetaW0(PI/2).thetaKp(3.0));
                    } else if (robot.isAtPose(110, 36, PI/2, 0.5, 0.5, PI/35) && robot.notMoving()) {
                        robot.drivetrain.stop();
                        knockStack = true;
                    } else {
                        robot.setTargetPoint(110, 36, PI/2);
                    }
                } else {
                    robot.intake.on();
                    robot.setTargetPoint(110, 36, PI/2);
                }

                if (time.seconds() > intakeStackTime) {
                    robot.shootYOverride = robot.y;
                    robot.highGoalShoot(2, true);

                    intakeStack = true;
                    time.reset();
                }
            }

            // Shoot 1 Ring in High Goal
            else if (!shoot1Ring) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    robot.intake.on();
                    shoot1Ring = true;
                    time.reset();
                }
            }

            // Intake Fourth Ring from Stack
            else if (!intakeStack2) {
                if (time.seconds() < 1.5) {
                    robot.setTargetPoint(new Target(110, Math.min(41 + 12 * time.seconds(), 63), PI/2).thetaW0(PI/2).thetaKp(3.0));
                } else {
                    robot.setTargetPoint(110, 63, PI/2);
                }

                if (time.seconds() > intakeStack2Time - 1) {
                    robot.shooter.flywheelHG();
                }

                if (time.seconds() > intakeStack2Time) {

                    // REMOVE after distance based velocity added
                    robot.thetaOffset = 0;
                    Constants.HIGH_GOAL_VELOCITY = highGoalVelocity;
                    //

                    robot.highGoalShoot();

                    intakeStack2 = true;
                    time.reset();
                }
            }

            // Shoot Rings in High Goal
            else if (!shootHighGoal2) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 30, 0, 0),
                            new Waypoint(wobbleCor[0], wobbleCor[1], wobbleCor[2], 5, -30, 0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    shootHighGoal2 = true;
                    time.reset();
                }
            }

            // Deliver Wobble Goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                robot.setTargetPoint(deliverWobblePath.getRobotPose(curTime));

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], wobbleCor[2])) || time.seconds() > deliverWobbleTime) {
                    robot.wobbleArm.armDown();
                }

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], wobbleCor[2]) && robot.notMoving()) || time.seconds() > deliverWobbleTime + 0.5) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > deliverWobbleTime + 1) {
                    robot.wobbleArm.armUp();

                    Waypoint[] parkWaypoints;
                    if (ringCase == RingCase.Zero) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                                new Waypoint(robot.x - 15, robot.y + 4, PI, 5, 5, 0, 0.75),
                                new Waypoint(110, 85, PI/2, 20, 10, 0, parkTime),
                        };
                    } else if (ringCase == RingCase.One) {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -20, -10, 0, 0),
                                new Waypoint(113, 85, PI/2, -20, -5, 0, parkTime),
                        };
                    } else {
                        parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, robot.theta, -50, -40, 0, 0),
                                new Waypoint(113, 85, PI/2, -30, -10, 0, parkTime),
                        };
                    }
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    deliverWobble = true;
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
