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

@Autonomous(name = "Blue Auto Power Shot", preselectTeleOp = "1 Teleop", group = "Blue")
public class BlueAutoPowerShot extends LinearOpMode {

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect stack
            shoot rings at powershot
            detect powershot bounce backs
            drop off wobble goal at corresponding zone
            collect powershot bounce backs
            shoot powershot bounce backs into high goal
            park on line
        */

        Robot robot = new Robot(this, 54, 9, PI/2, true, false);
        robot.logger.startLogging(true, false);

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        // Segments
        boolean align = false;
        boolean goToPowerShots = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean wobbleBack = false;
        boolean goToBounceBacks = false;
        boolean intakeBounceBacks = false;
        boolean goToBounceShoot = false;
        boolean shootBounceBacks = false;
        boolean park = false;

        // Segment Times
        double goToPowerShotsTime = 1.5;
        double deliverWobbleTime = 2.0;
        double wobbleBackTime = 1.0;
        double goToBounceBackTime = 2.5;
        double bounceBackTime = 4.0;
        double goToBounceShootTime = 2;
        double parkTime = 1.5;

        // Paths
        Waypoint[] goToPowerShotsWaypoints = new Waypoint[] {
                new Waypoint(54, 17, PI/2, 20, 20, 0, 0),
                new Waypoint(57, 63, PI/2, 5, -20, 0, goToPowerShotsTime),
        };
        Path goToPowerShotsPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShotsWaypoints)));
        Path deliverWobblePath = null;
        Path wobbleBackPath = null;
        Path goToBounceBackPath = null;
        double[] targetPoint = null;
        Path bounceBackPath = null;
        Path bounceBackThPath = null;
        Path goToBounceShootPath = null;
        Path parkPath = null;

        // Other Variables
        int depositState = 0;
        double depositReachTime = 0;
        ArrayList<Ring> rings;
        boolean resetCalled = false;

        waitForStart();

        robot.drivetrain.updateThetaError();
        robot.wobbleArm.armUp();

        // Determine Ring Case
        RingCase ringCase = RingCase.Zero;

        // Customize Pathing Depending on Ring Case
        double[][] wobbleDelivery = {{30, 84, 3*PI/2}, {55, 104, PI/2}, {15, 114, 7*PI/6}};
        double[] wobbleCor = wobbleDelivery[0];

        robot.setLockMode(Robot.TurretMode.HIGH_GOAL);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            rings = detector.getRingPipe().getRings(robot.x, robot.y, robot.theta);
            robot.ringPos = rings;

            if (!align) {
                robot.setTargetPoint(54, 17, 2*PI/3);

                if (time.seconds() > 0.5 && !resetCalled) {
                    robot.turretReset = true;
                    resetCalled = true;
                }

                if (time.seconds() > 1.5) {
                    ringCase = detector.getStackPipe().getModeResult();
                    Robot.log("Ring case: " + ringCase);

                    if (ringCase == RingCase.Zero) {
                        wobbleCor = wobbleDelivery[0];
                    } else if (ringCase == RingCase.One) {
                        wobbleCor = wobbleDelivery[1];
                    } else {
                        wobbleCor = wobbleDelivery[2];
                    }

                    detector.setPipeline(Vision.Pipeline.RingLocator);

                    robot.intake.blockerVert();

                    align = true;
                    time.reset();
                }
            }

            // Go to Power Shots
            else if (!goToPowerShots) {
                double curTime = Math.min(time.seconds(), goToPowerShotsTime);
                Pose curPose = goToPowerShotsPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                robot.shooter.setFlywheelVelocity(1550);

                if (time.seconds() > goToPowerShotsTime && robot.isAtPose(57, 63) && robot.notMoving()) {
                    robot.highGoalShoot(3, 1550);

                    goToPowerShots = true;
                    time.reset();
                }
            }

            // Shoot Power Shots
            else if (!shootPowerShots) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    robot.intake.bumpersOut();

                    Waypoint[] deliverWobbleWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 30, 0, 0),
                            new Waypoint(wobbleCor[0], wobbleCor[1], wobbleCor[2], 5, -30, 0, deliverWobbleTime),
                    };
                    deliverWobblePath = new Path(new ArrayList<>(Arrays.asList(deliverWobbleWaypoints)));

                    shootPowerShots = true;
                    time.reset();
                }
            }

            // Deliver Wobble Goal
            else if (!deliverWobble) {
                double curTime = Math.min(time.seconds(), deliverWobbleTime);
                if (ringCase == RingCase.Four) {
                    robot.setTargetPoint(deliverWobblePath.getRobotPose(curTime));
                } else {
                    robot.setTargetPoint(new Target(deliverWobblePath.getRobotPose(curTime)).theta(3*PI/2));
                }

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

                    if (ringCase != RingCase.Four) {
                        goToBounceBacks = true;
                        intakeBounceBacks = true;
                        goToBounceShoot = true;
                    }

                    targetPoint = new double[] {robot.x, robot.y};

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to Bounce Backs
            else if (!goToBounceBacks) {
                double curTime = Math.min(time.seconds(), goToBounceBackTime);
                if (curTime < (ringCase == RingCase.Zero ? 1 : 0.5)) {
                    robot.setTargetPoint(targetPoint[0] + 10, targetPoint[1], PI/2);
                } else {
                    robot.setTargetPoint(25, 129, PI/4);
                }

                robot.intake.on();

                if (time.seconds() > goToBounceBackTime || ringCase == RingCase.Four) {
                    Waypoint[] bounceBackWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, 0, 60, 60, 0, 0),
                            new Waypoint(35, 130, 0, 30, 20, 0, 3.0),
                            new Waypoint(62, 130, 0, 20, 5, 0, bounceBackTime),
                    };
                    bounceBackPath = new Path(new ArrayList<>(Arrays.asList(bounceBackWaypoints)));

                    Waypoint[] bounceBackThWaypoints = new Waypoint[] {
                            new Waypoint(PI/4, 0, 0, 0, 0, 0, 0),
                            new Waypoint(PI/4, 0, 0, 0, 0, 0, 3.5),
                            new Waypoint(PI/2, 0, 0, 0, 0, 0, bounceBackTime),
                    };
                    bounceBackThPath = new Path(new ArrayList<>(Arrays.asList(bounceBackThWaypoints)));

                    goToBounceBacks = true;
                    time.reset();
                }
            }

            // Intake Bounce Backs
            else if (!intakeBounceBacks) {
                double curTime = Math.min(time.seconds(), bounceBackTime);
                Pose curPose = bounceBackPath.getRobotPose(curTime);
                double theta = bounceBackThPath.getRobotPose(curTime).x;
                robot.setTargetPoint(new Target(curPose).theta(theta));

                if (time.seconds() > bounceBackTime) {
                    robot.intake.blockerHome();

                    Waypoint[] goToBounceShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -40, -30, 0, 0),
                            new Waypoint(60, 63, PI/2, -5, 20, 0, goToBounceShootTime),
                    };
                    goToBounceShootPath = new Path(new ArrayList<>(Arrays.asList(goToBounceShootWaypoints)));

                    intakeBounceBacks = true;
                    time.reset();
                }
            }

            // Go to High Goal
            else if (!goToBounceShoot) {
                double curTime = Math.min(time.seconds(), goToBounceShootTime);
                Pose curPose = goToBounceShootPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.x, curPose.y, curPose.theta + PI);

                if (time.seconds() > goToBounceBackTime/2) {
                    robot.intake.off();
                    robot.shooter.setFlywheelVelocity(1550);
                }

                if (time.seconds() > goToBounceShootTime) {
                    robot.highGoalShoot(3, 1550);

                    goToBounceShoot = true;
                    time.reset();
                }
            }

            // Shoot Bounce Backs
            else if (!shootBounceBacks) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    Waypoint[] parkWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                            new Waypoint(57, 85, PI/2, 5, 0, 0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    robot.wobbleArm.armUp();
                    robot.intake.bumpersHome();

                    shootBounceBacks = true;
                    time.reset();
                }
            }

            // Park
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                if (ringCase == RingCase.One) {
                    robot.setTargetPoint(new Target(parkPath.getRobotPose(curTime)).theta(PI/2));
                } else {
                    robot.setTargetPoint(parkPath.getRobotPose(curTime));
                }

                if (time.seconds() > parkTime) {
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
