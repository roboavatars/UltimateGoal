package org.firstinspires.ftc.teamcode.AutoPrograms.Red;

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

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

@Autonomous(name = "Red Auto Power Shot", preselectTeleOp = "1 Teleop", group = "Red")
public class RedAutoPowerShot extends LinearOpMode {

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

        Robot robot = new Robot(this, 90, 9, PI/2, true, true);
        robot.logger.startLogging(true, true);

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        // Segments
        boolean align = false;
        boolean goToPowerShots = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean goToBounceBacks = false;
        boolean intakeBounceBacks = false;
        boolean goToBounceShoot = false;
        boolean shootBounceBacks = false;
        boolean park = false;

        // Segment Times
        double goToPowerShotsTime = 1.5;
        double deliverWobbleTime = 2.0;
        double goToBounceBackTime = 2.5;
        double bounceBackTime = 4.0;
        double goToBounceShootTime = 2;
        double parkTime = 1.5;

        // Paths
        Waypoint[] goToPowerShotsWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 20, 15, 0, 0),
                new Waypoint(87, 63, PI/2, 5, -20, 0, goToPowerShotsTime),
        };
        Path goToPowerShotsPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShotsWaypoints)));
        Path deliverWobblePath = null;
        Path goToBounceBackPath = null;
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
        robot.intake.blockerVert();
        robot.wobbleArm.armUp();

        // Determine Ring Case
        RingCase ringCase = detector.getStackPipe().getModeResult();
        Robot.log("Ring case: " + ringCase);

        // Customize Pathing Depending on Ring Case
        double[][] wobbleDelivery = {{109, 86, PI/2}, {84, 102, PI/2}, {119, 126, 5*PI/8}};
        double[] wobbleCor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
        } else {
            wobbleCor = wobbleDelivery[2];
        }

        detector.setPipeline(Vision.Pipeline.RingLocator);
        robot.setLockMode(Robot.TurretMode.PS_R);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            rings = detector.getRingPipe().getRings(robot.x, robot.y, robot.theta);
            robot.ringPos = rings;

            if (!align) {
                robot.setTargetPoint(90, 17, PI/2);

                if (time.seconds() > 0.5 && !resetCalled) {
                    robot.turretReset = true;
                    resetCalled = true;
                }

                if (time.seconds() > 2) {
                    align = true;
                    time.reset();
                }
            }

            // Go to Power Shots
            else if (!goToPowerShots) {
                double curTime = Math.min(time.seconds(), goToPowerShotsTime);
                Pose curPose = goToPowerShotsPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                robot.shooter.flywheelPS();

                if (time.seconds() > goToPowerShotsTime && robot.isAtPose(87, 63) && robot.notMoving()) {
                    robot.powerShotShoot();

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
                robot.setTargetPoint(deliverWobblePath.getRobotPose(curTime));

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

                    Waypoint[] goToBounceBackWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta,  10, 5, 0, 0),
                            new Waypoint(robot.x, robot.y + 10, robot.theta, 10, 5, 0, 0.5),
                            new Waypoint(wobbleDelivery[2][0], wobbleDelivery[2][1], 3*PI/4, 5, 5, 0, goToBounceBackTime),
                    };
                    goToBounceBackPath = new Path(new ArrayList<>(Arrays.asList(goToBounceBackWaypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to Bounce Backs
            else if (!goToBounceBacks) {
                double curTime = Math.min(time.seconds(), goToBounceBackTime);
                robot.setTargetPoint(goToBounceBackPath.getRobotPose(curTime));

                robot.intake.on();

                if (time.seconds() > goToBounceBackTime || ringCase == RingCase.Four) {

                    Waypoint[] bounceBackWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, PI, 60, 60, 0, 0),
                            new Waypoint(87, 129, PI, 30, 20, 0, 3.0),
                            new Waypoint(82, 129, PI, 20, 5, 0, bounceBackTime),
                    };
                    bounceBackPath = new Path(new ArrayList<>(Arrays.asList(bounceBackWaypoints)));

                    Waypoint[] bounceBackThWaypoints = new Waypoint[] {
                            new Waypoint(3*PI/4, 0, 0, 0, 0, 0, 0),
                            new Waypoint(3*PI/4, 0, 0, 0, 0, 0, 3.0),
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
                    robot.setLockMode(Robot.TurretMode.HIGH_GOAL);

                    Waypoint[] goToBounceShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -40, -30, 0, 0),
                            new Waypoint(85, 63, PI/2, -5, 20, 0, goToBounceShootTime),
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

                if (time.seconds() > goToBounceBackTime / 2) {
                    robot.intake.off();
                    robot.shooter.setFlywheelVelocity(robot.calcHGVelocity());
                    robot.intake.blockerVert();
                }

                if (time.seconds() > goToBounceShootTime) {
                    robot.highGoalShoot();

                    goToBounceShoot = true;
                    time.reset();
                }
            }

            // Shoot Bounce Backs
            else if (!shootBounceBacks) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
                    Waypoint[] parkWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 20, 20, 0, 0),
                            new Waypoint(87, 85, PI/2, 5, 0, 0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                    robot.intake.bumpersHome();

                    shootBounceBacks = true;
                    time.reset();
                }
            }

            // Park
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                robot.setTargetPoint(parkPath.getRobotPose(curTime));

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
