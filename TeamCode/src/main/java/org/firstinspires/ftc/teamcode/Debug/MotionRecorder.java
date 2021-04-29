package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp @Disabled
public class MotionRecorder extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 9, 111, 0, false);
        robot.logger.startLogging(false);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }

        robot.logger.stopLogging();
    }
}