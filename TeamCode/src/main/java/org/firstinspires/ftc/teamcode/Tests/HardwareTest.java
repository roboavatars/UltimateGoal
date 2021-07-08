package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class HardwareTest extends LinearOpMode {

    public static String servoName1 = "leftClamp";
    public static String servoName2 = "rightClamp";

    public static double home1 = 0.2;
    public static double out1 = 0.6;
    public static double home2 = 0.5;
    public static double out2 = 0.05;

    public static boolean home = true;

    @Override
    public void runOpMode() {
        Servo servo1 = hardwareMap.get(Servo.class, servoName1);
        Servo servo2 = hardwareMap.get(Servo.class, servoName2);

        waitForStart();

        while(opModeIsActive()) {

            if (home) {
                servo1.setPosition(home1);
                servo2.setPosition(home2);
            } else {
                servo1.setPosition(out1);
                servo2.setPosition(out2);
            }

            if (gamepad1.dpad_left) {
                servo1.setPosition(home1);
                servo2.setPosition(home2);
            } else if (gamepad1.dpad_right) {
                servo1.setPosition(out1);
                servo2.setPosition(out2);
            }
        }
    }
}