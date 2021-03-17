package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

import com.acmerobotics.dashboard.config.Config;

import static java.lang.Math.PI;

@Config
@TeleOp(name = "PD Controller Test")
@Disabled
public class PDTest extends LinearOpMode {

    private Robot robot;
    private static double targetX = 96;
    private static double targetY = 72;
    private static double targetTheta = PI/2;
    public static double xKp = 0.6;
    public static double yKp = 0.6;
    public static double thetaKp = 4.5;
    public static double xKd = 0.05;
    public static double yKd = 0.05;
    public static double thetaKd = 0.3;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 87, 63, PI/2, false);
        robot.intake.sticksOut();
        robot.intake.updateSticks();

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                robot.resetOdo(87, 63, PI/2);
            }

            if (gamepad1.dpad_left) {
                robot.setTargetPoint(71, 72, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_right) {
                robot.setTargetPoint(121, 72, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_up) {
                robot.setTargetPoint(96, 122, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_down) {
                robot.setTargetPoint(96, 22, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.left_bumper) {
                robot.setTargetPoint(96, 72, PI, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.right_bumper) {
                robot.setTargetPoint(96, 72, 0, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.a) {
                robot.setTargetPoint(targetX, targetY, targetTheta, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();

//            addPacket("x", robot.x);
//            addPacket("y", robot.y);
//            addPacket("theta", robot.theta);
//            addPacket("vx", robot.vx);
//            addPacket("vy", robot.vy);
//            addPacket("w", robot.w);
//            sendPacket();
        }
    }
}