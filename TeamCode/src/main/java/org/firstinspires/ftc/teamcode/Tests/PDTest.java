package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import com.acmerobotics.dashboard.config.Config;

import static java.lang.Math.PI;

@Config
@TeleOp(name = "PD Controller Test")
public class PDTest extends LinearOpMode {

    private Robot robot;
    public static double targetX = 96;
    public static double targetY = 72;
    public static double targetTheta = PI/2;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 87, 63, PI/2, false);

        waitForStart();

        while(opModeIsActive()) {

//            if (gamepad1.dpad_left) {
//                robot.setTargetPoint(61, 72, PI/2, Robot.xKp, Robot.yKp, Robot.thetaKp, Robot.xKd, Robot.yKd, Robot.thetaKd);
//            } else if (gamepad1.dpad_right) {
//                robot.setTargetPoint(131, 72, PI/2, Robot.xKp, Robot.yKp, Robot.thetaKp, Robot.xKd, Robot.yKd, Robot.thetaKd);
//            } else if (gamepad1.dpad_up) {
//                robot.setTargetPoint(96, 132, PI/2, Robot.xKp, Robot.yKp, Robot.thetaKp, Robot.xKd, Robot.yKd, Robot.thetaKd);
//            } else if (gamepad1.dpad_down) {
//                robot.setTargetPoint(96, 12, PI/2, Robot.xKp, Robot.yKp, Robot.thetaKp, Robot.xKd, Robot.yKd, Robot.thetaKd);
//            } else if (gamepad1.left_bumper) {
//                robot.setTargetPoint(96, 72, PI, Robot.xKp, Robot.yKp, Robot.thetaKp, Robot.xKd, Robot.yKd, Robot.thetaKd);
//            } else if (gamepad1.right_bumper) {
//                robot.setTargetPoint(96, 72, 0, Robot.xKp, Robot.yKp, Robot.thetaKp, Robot.xKd, Robot.yKd, Robot.thetaKd);
//            } else if (gamepad1.a) {
//                robot.setTargetPoint(targetX, targetY, targetTheta, Robot.xKp, Robot.yKp, Robot.thetaKp, Robot.xKd, Robot.yKd, Robot.thetaKd);
//            } else {
//                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//            }

            robot.update();
        }
    }
}