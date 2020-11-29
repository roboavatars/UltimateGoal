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
    public static double home = 0.15;
    public static double out = 0.35;
    public static boolean isHome = true;
    private double position;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 90, 9, Math.PI/2, false);
        robot.shooter.magShoot();

        waitForStart();

        while(opModeIsActive()) {
            if (isHome) {
                position = home;
            } else {
                position = out;
            }

            if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            } else if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            }

            robot.update();
        }

        robot.stop();
    }
}