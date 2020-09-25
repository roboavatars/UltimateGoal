package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
public class Teleop extends LinearOpMode {

    public static int startX = 9;
    public static int startY = 111;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(startX, startY, 0, this);

        while(opModeIsActive()) {
            robot.update();
        }
    }
}
