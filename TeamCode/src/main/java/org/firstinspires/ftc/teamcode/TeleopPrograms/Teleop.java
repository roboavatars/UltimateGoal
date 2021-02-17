package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp
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

    @Override
    public void runOpMode() {
        if (useAutoPos) {
            double[] initialPosition = Logger.readPos();
            telemetry.addData("Starting Position", Arrays.toString(initialPosition)); telemetry.update();
            robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2], false);
        } else {
            robot = new Robot(this, startX, startY, startTheta, false);
        }

        robot.logger.startLogging();
        robot.intake.sticksOut();

        waitForStart();

        while (opModeIsActive()) {
            // Intake on/off/rev
            if (gamepad1.right_trigger > 0) {
                robot.intake.on();
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            // High goal and powershot shoot
            if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            } else if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            }

            if (gamepad1.dpad_left) {
                robot.highGoalShoot(2);
            }
            if (gamepad1.dpad_right) {
                robot.highGoalShoot(1);
            }

            // Rev up flywheel for high goal
            if (gamepad2.y) {
                robot.shooter.flywheelHG();
            }

            // Stop shoot sequence
            if (gamepad2.b) {
                robot.cancelShoot();
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
//            if (gamepad2.x && !stickToggle) {
//                stickToggle = true;
//                isStickAuto = !isStickAuto;
//            } else if (!gamepad2.x && stickToggle) {
//                stickToggle = false;
//            }
//
//            // Auto Mode Sticks
//            if (isStickAuto && robot.numRings == 0) {
//                robot.intake.autoSticks(robot.x, robot.y, robot.theta, 6);
//            } else if (!isStickAuto) {
//                robot.intake.sticksHome();
//            }

            // Ring blocker
            robot.intake.setBlocker(gamepad2.right_trigger);

            // Wobble motor controls
            robot.wobbleArm.setPower(-0.4 * gamepad2.left_stick_y);

            // Wobble clamp/unclamp
            if (gamepad2.a && !clampToggle) {
                clampToggle = true;
                if (clamped) {
                    robot.wobbleArm.unClamp();
                } else {
                    robot.wobbleArm.clamp();
                }
                clamped = !clamped;
            } else if (!gamepad2.a && clampToggle) {
                clampToggle = false;
            }

            // Slow align mode
            if (gamepad1.right_stick_button) {
                xySpeed = 0.3;
                thSpeed = 0.09;
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

            if (gamepad1.dpad_up) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapPosition() + 0.001);
            }
            if (gamepad1.dpad_down) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapPosition() - 0.001);
            }

            // Drivetrain controls
            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y * xySpeed, -gamepad1.left_stick_x * xySpeed, -gamepad1.right_stick_x * thSpeed);
            } else {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_y * xySpeed, gamepad1.left_stick_x * xySpeed, -gamepad1.right_stick_x * thSpeed);
            }

            // Update robot
            robot.update();

            // Telemetry
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("# Rings", robot.numRings);
            telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
            telemetry.update();
        }

        Robot.log("Cycles: " + robot.cycles);
        robot.shooter.feedHome();
        robot.stop();
    }
}