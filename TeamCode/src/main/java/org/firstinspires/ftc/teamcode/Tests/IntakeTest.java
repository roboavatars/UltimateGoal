package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;

@TeleOp(name = "Intake Test")
//@Disabled
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 108, 42, PI/2, false);
        robot.intake.setBlocker(0.5);
        ElapsedTime time = new ElapsedTime();

        waitForStart();

        time.reset();

        while (opModeIsActive()) {
            if (time.seconds() > 5.5) {
                break;
            } else if (time.seconds() > 5) {
                robot.intake.setPower(-0.4);
            } else if (time.seconds() > 4.5) {
                robot.intake.blockerDown();
            } else if (time.seconds() > 2.5) {
                robot.intake.on();
                robot.intake.setBlocker(0.95 - 0.175 * time.seconds());
                robot.drivetrain.stop();
            } else {
                robot.setTargetPoint(108, 42 + time.seconds(), PI/2);
            }

            robot.update();
        }
    }
}
