package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp @Config
public class Test extends LinearOpMode {

    public static double home = 0.24;
    public static double out = 0.495;
    public static boolean isHome = true;
    private double position;

    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.get(Servo.class, "magServo");

        waitForStart();

        while(opModeIsActive()) {
            if (isHome) {
                position = home;
            } else {
                position = out;
            }

            servo.setPosition(position);
        }
    }
}