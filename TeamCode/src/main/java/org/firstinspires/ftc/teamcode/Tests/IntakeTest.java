package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;

@TeleOp(name = "Intake Test")
@Config
//@Disabled
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 108, 42, PI/2, false);
        robot.intake.sticksCollect();
        robot.intake.setBlocker(0.31);
        ElapsedTime time = new ElapsedTime();

        waitForStart();
        time.reset();

        while (opModeIsActive()) {
            double input = Math.min(60, 42 + 4 * time.seconds() + 2.5 * Math.sin(8 * time.seconds()));
            robot.setTargetPoint(108, input, PI/2);

            if (time.seconds() > 5) {
                break;
            } else if (time.seconds() > 2.5) {
                robot.intake.reverse();
                robot.intake.setBlocker(0.41);
            } else if (time.seconds() > 0.75) {
                robot.intake.on();
                robot.intake.blockerDown();
            }

            robot.update();
        }
    }
}
