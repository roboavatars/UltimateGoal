package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@TeleOp
public class Teleop extends LinearOpMode {

    public static int startX = 135;
    public static int startY = 9;
    public static double startTheta = Math.PI/2;

    private Robot robot;
    private final boolean robotCentric = true;

    double targetShooterVelocity = 0;

    public boolean motorDown = false;
    public boolean clamped = false;

    @Override
    public void runOpMode() {

        /*double[] initialPosition = Logger.readPos();
        telemetry.addData("Starting Position", Arrays.toString(initialPosition));
        telemetry.update();*/

        robot = new Robot(this, startX, startY, startTheta); // Robot(this, initialPosition[0], initialPosition[1], initialPosition[2])
        // robot.logger.startLogging();
//        robot.intake.intakeOn();
        robot.t265.startCam();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.a) {
                robot.t265.reset(startX, startY, startTheta);
            }

            if (gamepad1.b) {
                robot.intake.intakeRev();
            } else {
                robot.intake.intakeOn();
            }

            if (gamepad1.left_bumper) {
                robot.shooter.magShoot();
                robot.shooter.setShooterVelocity(-2000);
            } else {
                robot.shooter.magHome();
                robot.shooter.flywheelOff();
            }

            if (gamepad1.right_bumper) {
                robot.shooter.feedShoot();
            } else {
                robot.shooter.feedHome();
            }

//            if (gamepad1.x) {
//                targetShooterVelocity += 1;
//                robot.shooter.setShooterVelocity(targetShooterVelocity);
//            } else if (gamepad1.b) {
//                targetShooterVelocity -= 1;
//                robot.shooter.setShooterVelocity(targetShooterVelocity);
//            }

            if (gamepad2.a && !motorDown) {
                robot.intake.wobbleDown();
                motorDown = true;
            } else if (gamepad2.a && motorDown) {
                robot.intake.wobbleUp();
                motorDown = false;
            }

            if (gamepad2.b && !clamped) {
                robot.intake.wobbleClamp();
                clamped = true;
            } else if (gamepad2.b && clamped) {
                robot.intake.wobbleRelease();
                clamped = false;
            }

            if (gamepad1.dpad_up && robot.shooter.getFlapAngle() < 0.22) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() + 0.001);
            }
            if (gamepad1.dpad_down && robot.shooter.getFlapAngle() > 0.03) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() - 0.001);
            }

            if (gamepad1.right_trigger > 0) {
                double[] target = robot.shoot(3);
                robot.setTargetPoint(target[0], target[1], target[2]);
                robot.shooter.setFlapAngle(target[3]);
            } else if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();

            telemetry.addData("Shooter Velocity", robot.shooter.getShooterVelocity());
            telemetry.addData("Target Shooter Velocity", targetShooterVelocity);
            telemetry.addData("Flap Angle", robot.shooter.getFlapAngle());
            telemetry.addData("Robot X", robot.drivetrain.x);
            telemetry.addData("Robot Y", robot.drivetrain.y);
            telemetry.addData("Robot Theta", robot.drivetrain.theta);
            telemetry.update();

        }
        // robot.logger.stopLogging();
        robot.t265.stopCam();
    }
}