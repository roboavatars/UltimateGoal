package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
public class Teleop extends LinearOpMode {

    public static int startX = 9;
    public static int startY = 111;

    private final boolean robotCentric = true;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(startX, startY, 0, this);

        waitForStart();

        while(opModeIsActive()) {

            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();
        }
    }
}
