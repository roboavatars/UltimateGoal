package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp @Config
public class Teleop extends LinearOpMode {

    public int startX = 90;
    public int startY = 9;
    public double startTheta = Math.PI/2;

    private Robot robot;
    public static boolean robotCentric = false;
    public static boolean useAutoPos = true;

    // Toggles
    public boolean flywheelToggle = false;
    public boolean flywheelOn = false;
    public boolean psToggle = false;
    public boolean ps = false;
    public boolean stickToggle = false;
    public boolean sticksOut = true;
    public boolean clampToggle = true;
    public boolean clamped = false;

    @Override
    public void runOpMode() {

        if (useAutoPos) {
            double[] initialPosition = Logger.readPos();
            telemetry.addData("Starting Position", Arrays.toString(initialPosition)); telemetry.update();
            robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2], false);
        } else {
            robot = new Robot(this, startX, startY, startTheta, false);
        }

        robot.wobbleArm.wobbleDown();
        robot.logger.startLogging();
        robot.intake.sticksOut();

        waitForStart();

        while (opModeIsActive()) {

            // Intake on/off/rev
            if (gamepad2.right_bumper) {
                robot.intake.intakeOn();
            } else if (gamepad2.left_bumper) {
                robot.intake.intakeRev();
            } else if (!robot.vibrateMag) {
                robot.intake.intakeOff();
            }

            // Auto high goal and powershot shoot
            if (gamepad1.right_bumper) {
                if (robot.shooter.getShooterVelocity() > 800) {
                    robot.powerShotShoot();
                }
            } else if (gamepad1.left_bumper) {
                if (robot.shooter.getShooterVelocity() > 800) {
                    robot.highGoalShoot();
                }
            }

            // Shoot x offset
            if (gamepad1.dpad_left) {
                robot.xOffset -= 0.01;
            } else if (gamepad1.dpad_right) {
                robot.xOffset += 0.01;
            } else if (gamepad1.dpad_up) {
                robot.xOffset = 0;
            }

            // Rev up flywheel for high goal
            if (gamepad2.y) {
                robot.shooter.flywheelHighGoal();
            }

            // Manual powershot controls
            if (gamepad2.right_trigger > 0 && !psToggle) {
                psToggle = true;
                if (ps) {
                    robot.shooter.flywheelOff();
                    robot.shooter.magHome();
                } else {
                    robot.shooter.flywheelPowershot();
                    robot.shooter.magShoot();
                }
                ps = !ps;
            } else if (gamepad2.right_trigger == 0 && psToggle) {
                psToggle = false;
            }

            if (gamepad2.left_trigger > 0 && ps) {
                robot.shooter.feedShoot();
            } else if (gamepad2.left_trigger == 0&& ps) {
                robot.shooter.feedHome();
            }

            // Stick extension/retraction
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

            // Wobble motor controls
            robot.wobbleArm.setPower(-0.5 * gamepad2.left_stick_y);

            // Wobble clamp/unclamp
            if (gamepad2.a && !clampToggle) {
                clampToggle = true;
                if (clamped) {
                    robot.wobbleArm.wobbleRelease();
                } else {
                    robot.wobbleArm.wobbleClamp();
                }
                clamped = !clamped;
            } else if (!gamepad2.a && clampToggle) {
                clampToggle = false;
            }

            // Mag vibrate to unstuck rings
            if (gamepad2.b) {
                robot.vibrateMag = true;
                robot.vibrateTime = System.currentTimeMillis();
            }

            // Drivetrain controls
            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            // Update robot
            robot.update();

            // Telemetry
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            double d = Math.sqrt(Math.pow(robot.x - 108, 2) + Math.pow(robot.y - 150, 2));
            telemetry.addData("High Goal Distance", d);
            telemetry.addData("# Rings", robot.numRings);
            telemetry.addData("Shooter Velocity", robot.shooter.getShooterVelocity());
            telemetry.update();
        }

        robot.shooter.feedHome();
        robot.stop();
    }
}