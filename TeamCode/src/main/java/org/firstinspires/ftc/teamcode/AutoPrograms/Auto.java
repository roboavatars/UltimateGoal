package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 9, 111, 0);

        waitForStart();

        while(opModeIsActive()) {
            robot.update();
        }
    }
}
