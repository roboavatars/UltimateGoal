package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
public class Regression extends LinearOpMode {

    public int startX = 90;
    public int startY = 9;
    public double startTheta = Math.PI/2;

    int shooterVelocity = -2000;

    private Robot robot;
    private final boolean robotCentric = false;

    @Override
    public void runOpMode() {

        robot = new Robot(this, startX, startY, startTheta);
        robot.t265.startCam();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.shooter.magShoot();
                robot.shooter.setShooterVelocity(shooterVelocity);
            } else {
                robot.shooter.magHome();
                robot.shooter.flywheelOff();
            }

            if (gamepad1.right_bumper) {
                robot.shooter.feedShoot();
            } else {
                robot.shooter.feedHome();
            }

//            if (gamepad1.a) {
//                shooterVelocity += 1;
//                robot.shooter.setShooterVelocity(shooterVelocity);
//            } else if (gamepad1.b) {
//                shooterVelocity -= 1;
//                robot.shooter.setShooterVelocity(shooterVelocity);
//            }

            if (gamepad1.dpad_up && robot.shooter.getFlapAngle() < 0.23) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() + 0.001);
            }
            if (gamepad1.dpad_down && robot.shooter.getFlapAngle() > 0.02) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() - 0.001);
            }

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
            telemetry.addData("Shooter Velocity", robot.shooter.getShooterVelocity());
            telemetry.addData("Flap Angle", robot.shooter.getFlapAngle());
            telemetry.update();

        }
        // robot.logger.stopLogging();
        robot.t265.stopCam();
    }
}