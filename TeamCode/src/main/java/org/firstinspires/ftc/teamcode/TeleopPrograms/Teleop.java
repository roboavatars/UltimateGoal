package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
public class Teleop extends LinearOpMode {

    public int startX = 90;
    public int startY = 9;
    public double startTheta = Math.PI/2;

    private Robot robot;
    private final boolean robotCentric = false;

    public boolean flywheelToggle = false;
    public boolean flywheelOn = false;
    public boolean stickToggle = false;
    public boolean sticksOut = true;
    public boolean motorToggle = false;
    public boolean clampToggle = false;
    public boolean motorDown = false;
    public boolean clamped = false;

    @Override
    public void runOpMode() {

        /*double[] initialPosition = Logger.readPos();
        telemetry.addData("Starting Position", Arrays.toString(initialPosition));
        telemetry.update();
        robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2])
        robot.logger.startLogging();*/

        robot = new Robot(this, startX, startY, startTheta); // Robot(this, initialPosition[0], initialPosition[1], initialPosition[2])
        robot.t265.startCam();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                robot.intake.intakeRev();
            } else {
                robot.intake.intakeOn();
            }

            if (gamepad1.right_bumper) {
                if (Math.abs(robot.shooter.getShooterVelocity() + 500) < 10) {
                    robot.powerShotShoot();
                } else {
                    robot.shooter.setShooterVelocity(-1000);
                }
            } else if (gamepad1.left_bumper) {
                if (Math.abs(robot.shooter.getShooterVelocity() + 2000) < 10) {
                    robot.highGoalShoot();
                } else {
                    robot.shooter.setShooterVelocity(-2000);
                }
            }

            if (gamepad2.y && !flywheelToggle) {
                flywheelToggle = true;
                if (flywheelOn) {
                    robot.shooter.flywheelOff();
                } else {
                    robot.shooter.flywheelOn();
                }
                flywheelToggle = !flywheelToggle;
            } else if (!gamepad2.y && flywheelToggle) {
                flywheelToggle = false;
            }

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

            if (gamepad2.a && !motorToggle) {
                motorToggle = true;
                if (motorDown) {
                    robot.intake.wobbleUp();
                } else {
                    robot.intake.wobbleDown();
                }
                motorDown = !motorDown;
            } else if (!gamepad2.a && motorToggle) {
                motorToggle = false;
            }

            if (gamepad2.b && !clampToggle) {
                clampToggle = true;
                if (clamped) {
                    robot.intake.wobbleRelease();
                } else {
                    robot.intake.wobbleClamp();
                }
                clamped = !clamped;
            } else if (!gamepad2.b && clampToggle) {
                clampToggle = false;
            }
//
//            if (gamepad1.dpad_up && robot.shooter.getFlapAngle() < 0.22) {
//                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() + 0.001);
//            }
//            if (gamepad1.dpad_down && robot.shooter.getFlapAngle() > 0.03) {
//                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() - 0.001);
//            }

            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();

            telemetry.addData("Robot X", robot.x);
            telemetry.addData("Robot Y", robot.y);
            telemetry.addData("Robot Theta", robot.theta);
            telemetry.addData("numRings", robot.numRings);
            telemetry.addData("Flap Angle", robot.shooter.getFlapAngle());
            telemetry.update();

        }

        robot.shooter.feedHome();

        // robot.logger.stopLogging();
        robot.t265.stopCam();
    }
}