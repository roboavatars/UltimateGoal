package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
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
//    public boolean isStickAuto = true;
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

            // Auto high goal and powershot shoot
            if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            } else if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            }

            // Rev up flywheel for high goal
            if (gamepad2.y) {
                robot.shooter.flywheelHG();
            }

            // Stop shoot sequence
            if (gamepad2.b) {
                robot.cancelShoot();
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

//            if (gamepad2.x && !stickToggle) {
//                stickToggle = true;
//                if (!stickToggle) {
//                    robot.intake.sticksHome();
//                    isStickAuto = false;
//                } else {
//                    isStickAuto = true;
//                }
//            } else if (!gamepad2.x && stickToggle) {
//                stickToggle = false;
//            }
//
//            // Sticks are in auto mode
//            if (isStickAuto && robot.numRings == 0) {
//                double x = robot.x;
//                double y = robot.y;
//                double theta = robot.theta;
//                double buffer = 6;
//                double[] leftPos = new double[] {x - 27 * Math.sin(theta) + 7 * Math.cos(theta), y + 27 * Math.cos(theta) + 7 * Math.sin(theta)};
//                double[] rightPos = new double[] {x + 27 * Math.sin(theta) + 7 * Math.cos(theta), y - 27 * Math.cos(theta) + 7 * Math.sin(theta)};
//                if (48 + buffer <= leftPos[0] && leftPos[0] <= 144 - buffer && 0 + buffer <= leftPos[1] && leftPos[1] <= 144 - buffer) {
//                    robot.intake.stickLeft(Constants.L_OUT_POS);
//                } else {
//                    robot.intake.stickLeft(Constants.L_HALF_POS);
//                }
//                if (48 + buffer <= rightPos[0] && rightPos[0] <= 144 - buffer && 0 + buffer <= rightPos[1] && rightPos[1] <= 144 - buffer) {
//                    robot.intake.stickRight(Constants.R_OUT_POS);
//                } else {
//                    robot.intake.stickRight(Constants.R_HALF_POS);
//                }
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
                robot.resetOdo(87, 63, Math.PI / 2);
                robot.xOffset = 0;
            }

            // Reset odo in corner
            if (gamepad2.left_trigger != 0) {
                robot.resetOdo(135, 9, Math.PI / 2);
                robot.xOffset = 0;
            }

            // Change shooting x offset to compensate for odo drift
            if (gamepad2.dpad_left) {
                robot.xOffset -= 0.1;
            } else if (gamepad2.dpad_right) {
                robot.xOffset += 0.1;
            }

            // Reset theta offset
            if (gamepad2.dpad_up) {
                robot.xOffset = 0;
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
            double d = Math.sqrt(Math.pow(robot.x - 108, 2) + Math.pow(robot.y - 150, 2));
            telemetry.addData("High Goal Distance", d);
            telemetry.addData("# Rings", robot.numRings);
            telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
            telemetry.update();
        }

        robot.shooter.feedHome();
        robot.stop();
    }
}