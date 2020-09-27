package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(9, 111, 0, this);

        waitForStart();

        while(opModeIsActive()) {
            robot.update();
        }
    }
}
