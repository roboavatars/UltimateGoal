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
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

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

        Robot robot = new Robot(this, 90, 9, PI/2, true);
        robot.logger.startLogging(true);

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        // Segments
        boolean goToPowerShots = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean goToBounceBacks = false;
        boolean intakeBounceBacks = false;
        boolean goToBounceShoot = false;
        boolean shootBounceBacks = false;
        boolean park = false;

        // Segment Times
        double goToPowerShotsTime = 2;
        double deliverWobbleTime = 2;
        double goToBounceBackTime = 2;
        double bounceBackTime = 5;
        double goToBounceShootTime = 2;
        double parkTime = 2;

        // Paths
        Waypoint[] goToPowerShotsWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 20, 30, 0, 0),
                new Waypoint(87, 63, PI/2, 5, -30, 0, goToPowerShotsTime),
        };
        Path goToPowerShotsPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShotsWaypoints)));
        Path deliverWobblePath = null;
        Path goToBounceBackPath = null;
        Path bounceBackPath = null;
        Path bounceBackThPath = null;
        Path goToBounceShootPath = null;
        Path parkPath = null;

        // Other Variables
        boolean reachedDeposit = false;
        double depositReachTime = 0;
        ArrayList<Ring> rings;

        waitForStart();

        robot.drivetrain.updateThetaError();

        // Determine Ring Case
        RingCase ringCase = detector.getStackPipe().getModeResult();
        Robot.log("Ring case: " + ringCase);

        // Customize Pathing Depending on Ring Case
        double[][] wobbleDelivery = {{115, 85, PI/2}, {90, 100, PI/2}, {125, 130, 2*PI/3}};
        double[] wobbleCor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDelivery[0];
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
        } else {
            wobbleCor = wobbleDelivery[2];
        }

        detector.setPipeline(Vision.Pipeline.RingLocator);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            rings = detector.getRingPipe().getRings(robot.x, robot.y, robot.theta);
            robot.ringPos = rings;

            // Go to Power Shots
            if (!goToPowerShots) {
                double curTime = Math.min(time.seconds(), goToPowerShotsTime);
                Pose curPose = goToPowerShotsPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

//                robot.shooter.flywheelPS();
                robot.shooter.flywheelHG();

                if (time.seconds() > goToPowerShotsTime) {
//                    robot.powerShotShoot();
                    robot.highGoalShoot();

                    goToPowerShots = true;
                    time.reset();
                }
            }

            // Shoot Power Shots
            else if (!shootPowerShots) {
                if (!robot.preShoot && !robot.shoot && robot.numRings == 0) {
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

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], wobbleCor[2])) || time.seconds() > deliverWobbleTime) {
                    robot.wobbleArm.armDown();
                }

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], wobbleCor[2]) && robot.notMoving()) || time.seconds() > deliverWobbleTime + 1) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 1) || time.seconds() > deliverWobbleTime + 3) {
                    robot.wobbleArm.armUp();

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

                if (time.seconds() > goToBounceBackTime || ringCase == RingCase.Four) {
                    Waypoint[] bounceBackWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, PI, 60, 60, 0, 0),
                            new Waypoint(92, 130, PI, 30, 20, 0, 2.75),
                            new Waypoint(82, 131, PI, 20, 5, 0, bounceBackTime),
                    };
                    bounceBackPath = new Path(new ArrayList<>(Arrays.asList(bounceBackWaypoints)));

                    Waypoint[] bounceBackThWaypoints = new Waypoint[] {
                            new Waypoint(robot.theta, 0, 0, 0, 0, 0, 0),
                            new Waypoint(3*PI/4, 0, 0, 0, 0, 0, 2.75),
                            new Waypoint(PI/2, 0, 0, 0, 0, 0, bounceBackTime),
                    };
                    bounceBackThPath = new Path(new ArrayList<>(Arrays.asList(bounceBackThWaypoints)));

                    robot.intake.on();

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
                    Waypoint[] goToBounceShootWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -40, -30, 0, 0),
                            new Waypoint(85, 63, PI/2, -5, 20, 0, goToBounceShootTime),
                    };
                    goToBounceShootPath = new Path(new ArrayList<>(Arrays.asList(goToBounceShootWaypoints)));

                    robot.intake.off();
                    robot.shooter.flywheelHG();

                    intakeBounceBacks = true;
                    time.reset();
                }
            }

            // Go to High Goal
            else if (!goToBounceShoot) {
                double curTime = Math.min(time.seconds(), goToBounceShootTime);
                Pose curPose = goToBounceShootPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose.x, curPose.y, curPose.theta + PI);

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
                            new Waypoint(robot.x, robot.y, robot.theta, 40, 30, 0, 0),
                            new Waypoint(85, 85, PI/2, 5, -30, 0, parkTime),
                    };
                    parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

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
