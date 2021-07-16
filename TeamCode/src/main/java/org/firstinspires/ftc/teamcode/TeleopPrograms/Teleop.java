package org.firstinspires.ftc.teamcode.TeleopPrograms;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

import static java.lang.Math.PI;

@TeleOp(name = "Not Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class Teleop extends LinearOpMode {

    // Backup Starting Position
    private final int startX = 111;
    private final int startY = 63;
    private final double startTheta = PI/2;

    private Robot robot;

    public static boolean robotCentric = true;
    public static boolean useAutoPos = false;

    // Control Gains
    private double xyGain = 1;
    private double wGain = 1;

    // Toggles
    private boolean downToggle = false, armDown = false;
    private boolean clampToggle = false, clamped = false;
    private boolean aimLockToggle = false, aimLock = false;
    private boolean magToggle = false, magUp = false;
    private boolean yIncreaseToggle = false, yDecreaseToggle = false;
    private boolean yOffsetToggle = false;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left stick/Right Stick - Drivetrain Controls
    X - Reset Odo
    B - Reset Odo Pods
    Left Bumper - High Goal Shoot
    Right Bumper - Powershot Shoot
    Left Trigger - Intake Reverse
    Right Trigger - Intake On

    Gamepad 2
    A - Blocker Up
    B - Cancel Shoot
    Y - Pre-Rev Flywheel for High Goal
    Dpad Left - Decrease Theta Offset
    Dpad Right - Increase Theta Offset
    Dpad Up - Increase Flap Angle
    Dpad Down - Decrease Flap Angle
    Left Bumper -  Wobble Clamp/Unclamp
    Right Bumper - Slow Mode
    Left Trigger - Shoot Override
    Right Trigger - Outake override
    Left Stick Button - Y Offset Decrease // Wobble Arm Up/Down
    Right Stick Button - Y Offset Increase // AimLock Toggle
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
        robot.wobbleArm.armUp();
        robot.setLockMode(Robot.TurretMode.HIGH_GOAL);

        waitForStart();

        while (opModeIsActive()) {
            // Intake On / Rev / Off
            if (gamepad1.right_trigger > 0 && gamepad2.right_trigger == 0) {
                if (robot.shooter.magHome) {
                    robot.intake.on();
                } else {
                    robot.intake.verticalOn();
                }
            } else if (gamepad1.left_trigger > 0 || gamepad2.right_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            // Override Shoots in Case it is Taking Too Long
            if (gamepad2.left_trigger > 0) {
                robot.preShootOverride = true;
                robot.shootOverride = true;
                if (!robot.preShoot && !robot.shoot) {
                    robot.highGoalShoot();
                }
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
            if (gamepad2.y) {
                robot.shooter.flywheelHG();
            }

            // Ring Blocker
            if (gamepad2.a) {
                robot.intake.blockerUp();
            } else {
                robot.intake.blockerDown();
            }

            // Wobble Arm Up / Down
            if (gamepad2.left_stick_button && !downToggle) {
                downToggle = true;
                if (armDown) {
                    robot.wobbleArm.armUp();
                } else {
                    robot.wobbleArm.armDown();
                }
                armDown = !armDown;
            } else if (!gamepad2.left_stick_button && downToggle) {
                downToggle = false;
            }

            // Wobble Clamp / Unclamp
            if (gamepad2.left_bumper && !clampToggle) {
                clampToggle = true;
                if (clamped) {
                    robot.wobbleArm.unClamp();
                } else {
                    robot.wobbleArm.clamp();
                }
                clamped = !clamped;
            } else if (!gamepad2.left_bumper && clampToggle) {
                clampToggle = false;
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
                robot.resetOdo(111, 63, PI/2);
                robot.thetaOffset = 0.05;
                robot.shootYOffset = 0;
                robot.flapOverride = 0;
            }

            /*if (gamepad2.right_stick_button && !yIncreaseToggle) {
                yIncreaseToggle = true;
                robot.shootYOffset += 3;
            } else if (!gamepad2.right_stick_button && yIncreaseToggle) {
                yIncreaseToggle = false;
            }

            if (gamepad2.left_stick_button && !yDecreaseToggle) {
                yDecreaseToggle = true;
                robot.shootYOffset -= 3;
            } else if (!gamepad2.left_stick_button && yDecreaseToggle) {
                yDecreaseToggle = false;
            }*/

            // Change Shooting Theta Offset to Compensate for Odometry Drift
            if (gamepad2.dpad_left) {
                robot.thetaOffset -= 0.01;
            } else if (gamepad2.dpad_right) {
                robot.thetaOffset += 0.01;
            }

            // Change Flap Angle
            if (gamepad2.dpad_up) {
                robot.flapOverride += 0.0025;
            } else if (gamepad2.dpad_down) {
                robot.flapOverride -= 0.0025;
            }

            // High goal lock
            if (gamepad2.right_stick_button && !aimLockToggle) {
                aimLockToggle = true;
                if (aimLock) {
                    robot.setLockMode(Robot.TurretMode.NONE);
                } else {
                    robot.setLockMode(Robot.TurretMode.HIGH_GOAL);
                }
                aimLock = !aimLock;
            } else if (!gamepad2.right_stick_button && aimLockToggle) {
                aimLockToggle = false;
            }

            /*if (gamepad1.a && !magToggle) {
                magToggle = true;
                if (magUp) {
                    robot.shooter.magHome();
                } else {
                    robot.shooter.magShoot();
                }
                magUp = !magUp;
            } else if (!gamepad1.a && magToggle) {
                magToggle = false;
            }*/

            // Drivetrain Controls
            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
            } else {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_y * xyGain, gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
            }

            // Update Robot
            robot.update();

            // Telemetry
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("# Rings", robot.numRings);
            telemetry.addData("Theta Offset", robot.thetaOffset);
            telemetry.addData("Shooter Velocity", robot.shooter.getFlywheelVelocity());
            telemetry.addData("# Cycles", robot.cycles);
            telemetry.addData("Average Cycle Time", (robot.cycleTotal / robot.cycles) + "s");
            telemetry.update();
        }

        Log.w("cycle-log", "# Cycles: " + robot.cycles);
        Log.w("cycle-log", "Avg cycle Time: " + (robot.cycleTotal / robot.cycles) + "s");
        if (robot.cycles > 0) {
            Log.w("cycle-log", "Avg dropping longest: " + ((robot.cycleTotal - robot.longestCycle) / (robot.cycles - 1)) + "s");
        }
        robot.stop();
    }
}