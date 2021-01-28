package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
//@Disabled
public class SideFlapTest extends LinearOpMode {

    public static double homePos = 0;
    public static double outPos = 0.34;
    public static boolean isHome = true;
    private double flapPos;

    @Override
    public void runOpMode() {
        Servo flapServo = hardwareMap.get(Servo.class, "flapServo");

        waitForStart();

        while(opModeIsActive()) {
            if (isHome) {
                flapPos = homePos;
            } else {
                flapPos = outPos;
            }

            flapServo.setPosition(flapPos);
        }
    }
}