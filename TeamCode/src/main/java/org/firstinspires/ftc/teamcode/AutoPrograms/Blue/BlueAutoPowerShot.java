package org.firstinspires.ftc.teamcode.AutoPrograms.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
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

        /*Robot robot = new Robot(this, 90, 9, PI/2, true);
        robot.logger.startLogging(true);

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        // Segments
        boolean goToPowerShots = false;
        boolean shootPowerShots = false;
        boolean deliverWobble = false;
        boolean goToBounceBacks = false;
        boolean intakeBounceBacks = false;
        boolean goToHighGoal = false;
        boolean shootBounceBacks = false;
        boolean park = false;

        // Segment Times
        double goToPowerShotsTime = 1.0;
        double deliverWobbleTime = 1.75;
        double goToBounceBacksTime = 1.25;
        double intakeBounceBacksTime = 3.5;
        double goToHighGoalTime = 1.5;
        double parkTime = 1.25;

        // Paths
        Waypoint[] goToPowerShotsWaypoints = new Waypoint[] {
                new Waypoint(90, 9, PI/2, 30, 30, 0, 0),
                new Waypoint(87, 63, PI/2, 5, -30, 0, goToPowerShotsTime),
        };
        Path goToPowerShotsPath = new Path(new ArrayList<>(Arrays.asList(goToPowerShotsWaypoints)));
        Path deliverWobblePath = null;
        Path goToBounceBacksPath = null;
        Path intakeBounceBacksPath = null;
        Path goToHighGoalPath = null;
        Path parkPath = null;

        // Other Variables
        boolean reachedDeposit = false;
        double depositReachTime = 0;
        ArrayList<Ring> rings;

        waitForStart();

        // Determine Ring Case
        RingCase ringCase = detector.getStackPipe().getModeResult();
        Robot.log("Ring case: " + ringCase);

        // Customize Pathing Depending on Ring Case
        double[][] wobbleDeliveryPositions = {{125, 69, PI/2}, {105, 93, PI/4}, {127, 117, PI}};
        double[] wobbleCor;
        if (ringCase == RingCase.Zero) {
            wobbleCor = wobbleDeliveryPositions[0];
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDeliveryPositions[1];
        } else {
            wobbleCor = wobbleDeliveryPositions[2];
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

                if (curTime > goToPowerShotsTime) {
                    robot.powerShotShoot();

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
                Pose curPose = deliverWobblePath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], wobbleCor[2])) || time.seconds() > 0.75) {
                    robot.wobbleArm.armDown();
                }

                if ((!reachedDeposit && robot.isAtPose(wobbleCor[0], wobbleCor[1], wobbleCor[2]) && robot.notMoving()) || time.seconds() > 1.25) {
                    reachedDeposit = true;
                    depositReachTime = curTime;
                    robot.wobbleArm.unClamp();
                }

                if ((reachedDeposit && time.seconds() > depositReachTime + 0.5) || time.seconds() > deliverWobbleTime + 0.5) {
                    robot.wobbleArm.armUp();

                    Waypoint[] goToBounceBacksWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, 30, 20, 0, 0),
                            new Waypoint(130, 127, 3*PI/4, 5, 5, 0, goToBounceBacksTime),
                    };
                    goToBounceBacksPath = new Path(new ArrayList<>(Arrays.asList(goToBounceBacksWaypoints)));

                    deliverWobble = true;
                    time.reset();
                }
            }

            // Go to Bounce Backs
            else if (!goToBounceBacks) {
                double curTime = Math.min(time.seconds(), goToBounceBacksTime);
                Pose curPose = goToBounceBacksPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                if (curTime > goToBounceBacksTime || ringCase == RingCase.Four) {
                    Waypoint[] intakeBounceBacksWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, PI, 60, 60, 0, 0),
                            new Waypoint(91, 127, PI, 30, 20, 0, 2.75),
                            new Waypoint(81, 131, PI, 20, 5, 0, intakeBounceBacksTime),
                    };
                    intakeBounceBacksPath = new Path(new ArrayList<>(Arrays.asList(intakeBounceBacksWaypoints)));

                    robot.intake.on();

                    goToBounceBacks = true;
                    time.reset();
                }
            }

            // Intake Bounce Backs
            else if (!intakeBounceBacks) {
                double curTime = Math.min(time.seconds(), intakeBounceBacksTime);
                Pose curPose = intakeBounceBacksPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                if (curTime > intakeBounceBacksTime) {
                    Waypoint[] goToHighGoalWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -40, -30, 0, 0),
                            new Waypoint(85, 63, PI/2, -5, 20, 0, goToHighGoalTime),
                    };
                    goToHighGoalPath = new Path(new ArrayList<>(Arrays.asList(goToHighGoalWaypoints)));

                    robot.intake.off();

                    intakeBounceBacks = true;
                    time.reset();
                }
            }

            // Go to High Goal
            else if (!goToHighGoal) {
                double curTime = Math.min(time.seconds(), goToHighGoalTime);
                Pose curPose = goToHighGoalPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

                if (curTime > intakeBounceBacksTime) {
                    robot.highGoalShoot();

                    goToHighGoal = true;
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
                Pose curPose = parkPath.getRobotPose(curTime);
                robot.setTargetPoint(curPose);

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
        } catch (Exception ignore) {}*/
    }
}
