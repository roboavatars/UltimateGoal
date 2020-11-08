package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@Config
@TeleOp
public class Teleop extends LinearOpMode {

    public static int startX = 9;
    public static int startY = 111;
    public static int startTheta = 0;

    private Robot robot;
    private final boolean robotCentric = true;

    @Override
    public void runOpMode() {
        /*double[] initialPosition = Logger.readPos();
        telemetry.addData("Starting Position", Arrays.toString(initialPosition));
        telemetry.update();*/

        robot = new Robot(this, startX, startY, startTheta); // Robot(this, initialPosition[0], initialPosition[1], initialPosition[2])
        // robot.logger.startLogging();
        robot.shooter.flywheelOn();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.a) {

            } else {

            }

            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();
        }
        // robot.logger.stopLogging();
    }
}
