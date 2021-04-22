package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;

@TeleOp(name = "Intake Test")
@Disabled
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 109, 38, PI/2, false);
        robot.intake.blockerUp();

        ElapsedTime time = new ElapsedTime();
        double intakeStackTime = 2.5;

        waitForStart();
        time.reset();

        while (opModeIsActive()) {
            if (time.seconds() > intakeStackTime) {
                break;
            } else if (time.seconds() > intakeStackTime - 0.75) {
                robot.intake.setPower(-0.75);
                robot.intake.blockerDown();
            } else if (robot.isAtPose(109, 38, PI/2, 0.5, 0.5, PI/35)) {
                robot.intake.on();
                robot.intake.setBlocker(0.51 - 0.12 * time.seconds());
                robot.drivetrain.stop();
            } else {
                robot.setTargetPoint(109, 38, PI/2);
            }

            robot.update();
        }
    }
}
