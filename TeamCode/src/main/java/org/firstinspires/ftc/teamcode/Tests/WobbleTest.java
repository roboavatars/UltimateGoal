package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.WobbleArm;

public class WobbleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        WobbleArm wobbleArm = new WobbleArm(this);

        waitForStart();

        while(opModeIsActive()) {
            wobbleArm.clamp();
        }

    }
}
