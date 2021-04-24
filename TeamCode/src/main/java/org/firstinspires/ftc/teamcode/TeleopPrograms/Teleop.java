package org.firstinspires.ftc.teamcode.TeleopPrograms;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

import static java.lang.Math.PI;

@TeleOp(name = "1 Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class Teleop extends LinearOpMode {

    // Backup Starting Position
    private final int startX = 111;
    private final int startY = 63;
    private final double startTheta = PI/2;

    private Robot robot;

    public static boolean robotCentric = true;
    public static boolean useAutoPos = true;
    public static boolean autoSticks = false;

    // Control Gains
    private double xyGain = 1;
    private double wGain = 1;

    // Toggles
    private boolean stickToggle = false;
    private boolean downToggle = false, armDown = false;
    private boolean clampToggle = false, clamped = false;
    private boolean aimLockToggle = false, aimLock = false;
    private boolean magToggle = false, magUp = false;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left stick/Right Stick - Drivetrain Controls
    X - Reset Odo
    Left Bumper - High Goal Shoot
    Right Bumper - Powershot Shoot
    Left Trigger - Intake Reverse
    Right Trigger - Intake On

    Gamepad 2
    A - Blocker Up
    B - Cancel Shoot
    X - Toggle Sticks (Up/Out)
    Y - Pre-Rev Flywheel for High Goal
    Dpad Left - Decrease Theta Offset
    Dpad Right - Increase Theta Offset
    Dpad Up - Reset Theta Offset
    Dpad Down - Wobble Arm Up/Down
    Left Bumper -  Wobble Clamp/Unclamp
    Right Bumper - Slow Mode
    Left Trigger - Shoot Override
    Right Trigger - Outake override
    Left Stick Button - Mag Up - Commented
    Right Stick Button - AimLock Toggle
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
        robot.intake.sticksOut();
        robot.wobbleArm.armUp();

        waitForStart();

        while (opModeIsActive()) {
            // Intake On / Rev / Off
            if (gamepad1.right_trigger > 0 && gamepad2.right_trigger == 0) {
                robot.intake.on();
            } else if (gamepad1.left_trigger > 0 || gamepad2.right_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            // Override Shoots in Case it is Taking Too Long
            robot.preShootOverride = gamepad2.left_trigger > 0;

            // High Goal / Powershot Shoot
            robot.aimLockShoot = aimLock;
            if (gamepad1.left_bumper) {
                robot.highGoalShoot(true);
            } else if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            }

            // Stop Shoot Sequence
            if (gamepad2.b) {
                robot.cancelShoot();
            }

            // Rev Up Flywheel for High Goal
            if (gamepad2.y /*|| (!robot.shooter.sensorBroken && robot.numRings == 2)*/) {
                robot.shooter.flywheelHG();
                robot.intake.sticksShoot();
            }

            // Auto Move Back Sticks After 3 Rings
            /*if (!robot.shooter.sensorBroken && robot.numRings == 3 && !robot.preShoot && !robot.shoot && !robot.postShoot) {
                robot.intake.sticksOut();
            }*/

            // Stick Retraction/Extension Toggle
            if (gamepad2.x && !stickToggle) {
                stickToggle = true;
                if (!robot.intake.sticksHalf) {
                    robot.intake.sticksHalf();
                } else {
                    robot.intake.sticksOut();
                }
            } else if (!gamepad2.x && stickToggle) {
                stickToggle = false;
            }

            if (autoSticks) {
                robot.intake.autoSticks(robot.x, robot.y, robot.theta, 2);
            }

            // Ring Blocker
            if (gamepad2.a) {
                robot.intake.blockerUp();
            } else {
                robot.intake.blockerDown();
            }

            // Wobble Arm Up / Down
            if (gamepad2.dpad_down && !downToggle) {
                downToggle = true;
                if (armDown) {
                    robot.wobbleArm.armUp();
                } else {
                    robot.wobbleArm.armDown();
                }
                armDown = !armDown;
            } else if (!gamepad2.dpad_down && downToggle) {
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

            // Enter Aimlock / Strafe Mode
            if (gamepad2.right_stick_button && !aimLockToggle) {
                aimLockToggle = true;
                aimLock = !aimLock;
            } else if (!gamepad2.right_stick_button && aimLockToggle) {
                aimLockToggle = false;
            }

            // Mag Up
//            if (gamepad2.left_stick_button) {
//                robot.shooter.magShoot();
//            } else if (!robot.preShoot && !robot.shoot) {
//                robot.shooter.magHome();
//            }

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
            if (!aimLock || gamepad2.right_bumper) {
                if (robotCentric) {
                    robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
                } else {
                    robot.drivetrain.setGlobalControls(gamepad1.left_stick_y * xyGain, gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
                }
            } else if (aimLock) {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_x * xyGain, MecanumDrivetrain.yKp * (63 - robot.y) + MecanumDrivetrain.yKd * (-robot.vy), (gamepad1.right_stick_x == 0 ? MecanumDrivetrain.thetaKp * (robot.shootTargets(3)[2] - robot.theta) + MecanumDrivetrain.thetaKd * (-robot.w) : -gamepad1.right_stick_x * 0.5 * wGain));
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
            telemetry.addData("AimLock", aimLock);
            telemetry.addData("# Rings", robot.numRings);
            telemetry.addData("Theta Offset", robot.thetaOffset);
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