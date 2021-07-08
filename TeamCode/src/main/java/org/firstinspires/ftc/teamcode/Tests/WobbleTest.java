package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.WobbleArm;

@TeleOp
@Config
@Disabled
public class WobbleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        WobbleArm wobbleArm = new WobbleArm(this);

        waitForStart();

        while(opModeIsActive()) {
            wobbleArm.unClamp();
            wobbleArm.armDown();
            sleep(1000);
            wobbleArm.clamp();
            sleep(1000);
            wobbleArm.armUp();
        }
    }
}
