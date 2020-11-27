package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
@Config
public class Test extends LinearOpMode {

    private Robot robot;
    public static double x = 90.08;
    public static double y = 68.99;
    public static double theta = 1.4725;
    public static double flap = 0.0380;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 90, 9, Math.PI/2, false);
        robot.wobbleArm.wobbleDown();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.a) {
                robot.setTargetPoint(x, y, theta, 0.17, 0.17, 1.9);
                robot.shooter.setFlapAngle(flap);
            } else {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            if (gamepad1.b) {
                robot.shooter.setShooterVelocity(-875);
            }

            if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            } else if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            }

            robot.update();

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Theta", theta);
            telemetry.addData("Flap", flap);
            telemetry.update();
        }

        robot.stop();
    }
}