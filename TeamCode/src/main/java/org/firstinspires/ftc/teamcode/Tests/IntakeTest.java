package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.outlineRect;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Intake Test")
@Config
//@Disabled
public class IntakeTest extends LinearOpMode {
    private Intake intake;

    public static double x = 90;
    public static double y = 9;
    public static double theta = PI/2;
    public static double buffer = 6;

    @Override
    public void runOpMode() {
        intake = new Intake(this, false);

        waitForStart();

        while (opModeIsActive()) {
            intake.autoBumpers(x, y, theta, buffer);
            intake.updateBumpers();

            drawDrivetrain(x, y, theta, "black");
            outlineRect(buffer, buffer, 144 - buffer, 144 - buffer, "black");
            sendPacket();
        }
    }
}
