package org.firstinspires.ftc.teamcode.TeleopPrograms;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp(name = "Teleop")
@Config
public class Teleop extends LinearOpMode {

    public int startX = 90;
    public int startY = 9;
    public double startTheta = Math.PI/2;

    private Robot robot;
    public static boolean robotCentric = true;
    public static boolean useAutoPos = true;

    public double xySpeed = 1;
    public double thSpeed = 1;

    // Toggles
    public boolean stickToggle = false;
    public boolean sticksOut = true;
    public boolean isStickAuto = true;
    public boolean clampToggle = false;
    public boolean clamped = true;
    public boolean aimLockToggle = false;
    public boolean aimLock = true;

    @Override
    public void runOpMode() {
        if (useAutoPos) {
            double[] initialPosition = Logger.readPos();
            telemetry.addData("Starting Position", Arrays.toString(initialPosition));
            telemetry.update();
            robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2], false);
        } else {
            robot = new Robot(this, startX, startY, startTheta, false);
        }

        robot.logger.startLogging(false);
        robot.intake.sticksOut();

        waitForStart();

        while (opModeIsActive()) {
            // Intake on/off/rev
            if (gamepad2.right_trigger > 0) {
                robot.intake.on();
                robot.intake.motor2Power(-gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0) {
                robot.intake.reverse();
                robot.intake.motor2Power(gamepad2.a ? 1 : 0.7);
            } else {
                robot.intake.off();
                robot.intake.motor2Power(0);
            }

            // High goal and powershot shoot
            if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            } else if (gamepad1.right_bumper) {
                robot.powerShotShoot();
                robot.intake.sticksFourth();
            }

            // Stop shoot sequence
            if (gamepad2.b) {
                robot.cancelShoot();
            }

            // Rev up flywheel for high goal
            if (gamepad2.y || (!robot.shooter.sensorBroken && robot.numRings == 2)) {
                robot.shooter.flywheelHG();
                robot.intake.sticksFourth();
            }

            // Auto raise sticks after 3 rings
            if (!robot.shooter.sensorBroken && robot.numRings == 3) {
                robot.intake.sticksFourth();
            }

            // Stick Retraction/Extension Toggle
            if (gamepad2.x && !stickToggle) {
                stickToggle = true;
                if (sticksOut) {
                    robot.intake.sticksHome();
                } else {
                    robot.intake.sticksOut();
                }
                sticksOut = !sticksOut;
            } else if (!gamepad2.x && stickToggle) {
                stickToggle = false;
            }

            // Stick Retraction/Auto Toggle
            /*if (gamepad2.x && !stickToggle) {
                stickToggle = true;
                isStickAuto = !isStickAuto;
            } else if (!gamepad2.x && stickToggle) {
                stickToggle = false;
            }

            // Auto Mode Sticks
            if (isStickAuto && robot.numRings == 0) {
                robot.intake.autoSticks(robot.x, robot.y, robot.theta, 6);
            } else if (!isStickAuto) {
                robot.intake.sticksHome();
            }*/

            // Ring blocker
            if (gamepad2.a) {
                robot.intake.blockerDown();
            } else {
                robot.intake.blockerUp();
            }

            // Wobble motor controls
            robot.wobbleArm.setPower(-0.4 * gamepad2.left_stick_y);

            // Wobble clamp/unclamp
            if (gamepad2.dpad_down && !clampToggle) {
                clampToggle = true;
                if (clamped) {
                    robot.wobbleArm.unClamp();
                } else {
                    robot.wobbleArm.clamp();
                }
                clamped = !clamped;
            } else if (!gamepad2.dpad_down && clampToggle) {
                clampToggle = false;
            }

            // Slow align mode
            if (gamepad2.left_trigger > 0) {
                xySpeed = 0.22;
                thSpeed = 0.17;
            } else {
                xySpeed = 1;
                thSpeed = 1;
            }

            // Reset odo for powershot
            if (gamepad1.x) {
                robot.resetOdo(87, 63, Math.PI/2);
                robot.thetaOffset = 0;
            }

            // Change shooting theta offset to compensate for odo drift
            if (gamepad2.dpad_left) {
                robot.thetaOffset -= 0.01;
            } else if (gamepad2.dpad_right) {
                robot.thetaOffset += 0.01;
            }

            // Reset theta offset
            if (gamepad2.dpad_up) {
                robot.thetaOffset = 0;
            }

            // Enter aimlock/strafe mode
            if (gamepad2.left_bumper && !aimLockToggle) {
                aimLockToggle = true;
                aimLock = !aimLock;
            } else if (!gamepad2.x && aimLockToggle) {
                aimLockToggle = false;
            }

            // Drivetrain controls
            if (gamepad2.a) {
                robot.setTargetPoint(87, 63, Math.PI/2);
            } else if (!aimLock || gamepad2.left_trigger > 0) {
                if (robotCentric) {
                    robot.drivetrain.setControls(-gamepad1.left_stick_y * xySpeed, -gamepad1.left_stick_x * xySpeed, -gamepad1.right_stick_x * thSpeed);
                } else {
                    robot.drivetrain.setGlobalControls(gamepad1.left_stick_y * xySpeed, gamepad1.left_stick_x * xySpeed, -gamepad1.right_stick_x * thSpeed);
                }
            } else if (!robot.preShoot && !robot.shoot) {
                robot.drivetrain.setGlobalControls(MecanumDrivetrain.yKp * (63 - robot.y) + MecanumDrivetrain.yKd * (-robot.vy), gamepad1.left_stick_y * xySpeed, MecanumDrivetrain.thetaKp * (robot.theta - robot.shootTargets(3)[2]) + MecanumDrivetrain.thetaKd * (-robot.w));
            }

            // Update robot
            robot.update();

            // Telemetry
            if (robot.shooter.sensorBroken) {
                telemetry.addData("Distance Sensor", "Broken");
            }
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("# Rings", robot.numRings);
            telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
            telemetry.addData("# Cycles", robot.cycles);
            telemetry.addData("Average Cycle Time", (robot.cycleTotal / robot.cycles) + "s");
            telemetry.update();
        }

        Log.w("cycle-log", "# Cycles: " + robot.cycles);
        Log.w("cycle-log", "Avg cycle Time: " + (robot.cycleTotal / robot.cycles) + "s");
        robot.stop();
    }
}