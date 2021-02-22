package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;

@Autonomous(name = "Intake Test")
@Disabled
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, 108, 42, PI/2, true);
        robot.intake.sticksHome();
        robot.intake.blockerDown();

        waitForStart();

        robot.intake.on();
        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            double input = Math.min(60, 42 + 6 * time.seconds() + 1.5 * Math.sin(9 * time.seconds()));
            robot.setTargetPoint(108, input, PI/2);

            robot.update();
        }

    }
}
