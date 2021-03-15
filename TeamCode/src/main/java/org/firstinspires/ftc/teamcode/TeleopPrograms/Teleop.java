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

    public double xyGain = 1;
    public double wGain = 1;

    // Toggles
    public boolean aimLockToggle = false;
    public boolean aimLock = false;

    /*
    Gamepad 1:
    Left Trigger - Intake On
    Right Trigger - Intake Reverse
    Left Bumper - High Goal Shoot
    Right Bumper - Powershot Shoot
    X - Odometry Reset
    Joysticks - Drivetrain Controls

    Gamepad 2:
    B - Cancel Shoot
    Y - Pre-Rev Flywheel for High Goal
    A - Blocker Up
    Left Trigger - Left Stick
    Right Trigger - Right Stick
    Right Bumper - Slow Mode
    Dpad Left - Decrease Theta Offset
    Dpad Right - Increase Theta Offset
    Dpad Up - Reset Theta Offset
    Left Bumper - Aimlock Toggle
     */

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
        robot.intake.sticksCollect();
        robot.wobbleArm.armUp();

        waitForStart();

        while (opModeIsActive()) {
            // Intake On / Rev / Off
            if (gamepad1.left_trigger > 0) {
                robot.intake.on();
            } else if (gamepad1.right_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            // High Goal / Powershot Shoot
            if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            } else if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            }

            // Stop Shoot Sequence
            if (gamepad2.b) {
                robot.cancelShoot();
            }

            // Rev Up Flywheel for High Goal
            if (gamepad2.y || (!robot.shooter.sensorBroken && robot.numRings == 2)) {
                robot.shooter.flywheelHG();
            }

            // Auto Move Back Sticks After 3 Rings
            if (!robot.shooter.sensorBroken && robot.numRings == 3 && !robot.preShoot && !robot.shoot && !robot.postShoot) {
                robot.intake.sticksOut();
            }

            // Ring Blocker
            if (gamepad2.a) {
                robot.intake.blockerUp();
            } else {
                robot.intake.blockerDown();
            }

            // Individual Stick Control
            if (!robot.preShoot && !robot.shoot && !robot.postShoot) {
                robot.intake.stickLeft(gamepad2.left_trigger);
                robot.intake.stickRight(1 - gamepad2.right_trigger);
            }

            // Slow Mode
            if (gamepad2.right_bumper) {
                xyGain = 0.22;
                wGain = 0.17;
            } else {
                xyGain = 1;
                wGain = 1;
            }

            // Reset Odometry
            if (gamepad1.x) {
                robot.resetOdo(111, 63, Math.PI/2);
                robot.thetaOffset = 0;
            }

            // Change Shooting Theta Offset to Compensate for Odometry Drift
            if (gamepad2.dpad_left) {
                robot.thetaOffset -= 0.01;
            } else if (gamepad2.dpad_right) {
                robot.thetaOffset += 0.01;
            } else if (gamepad2.dpad_up) {
                robot.thetaOffset = 0;
            }

            // Enter Aimlock/Strafe Mode
            if (gamepad2.left_bumper && !aimLockToggle) {
                aimLockToggle = true;
                aimLock = !aimLock;
            } else if (!gamepad2.left_bumper && aimLockToggle) {
                aimLockToggle = false;
            }

            // Drivetrain Controls
            if (!aimLock || gamepad2.right_bumper) {
                if (robotCentric) {
                    robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
                } else {
                    robot.drivetrain.setGlobalControls(gamepad1.left_stick_y * xyGain, gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
                }
            } else if (!robot.preShoot && !robot.shoot) {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_x * xyGain, MecanumDrivetrain.yKp * (63 - robot.y) + MecanumDrivetrain.yKd * (-robot.vy), MecanumDrivetrain.thetaKp * (robot.shootTargets(3)[2] - robot.theta) + MecanumDrivetrain.thetaKd * (-robot.w));
            }

            // Update Robot
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