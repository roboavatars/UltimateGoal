package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class HardwareTest extends LinearOpMode {

    public static double leftHome = 0.55;
    public static double rightHome = 0;
    public static double leftOut = 0.55;
    public static double rightOut = 0;
    public static boolean home = true;
    public static boolean debug = false;

    @Override
    public void runOpMode() {
        Servo leftStick = hardwareMap.get(Servo.class, "leftStick");
        Servo rightStick = hardwareMap.get(Servo.class, "rightStick");

        waitForStart();

        while(opModeIsActive()) {
            if (debug) {
                if (home) {
                    leftStick.setPosition(leftHome);
                    rightStick.setPosition(rightHome);
                } else {
                    leftStick.setPosition(leftOut);
                    rightStick.setPosition(rightOut);
                }
            } else {
                if (gamepad1.dpad_left) {
                    leftStick.setPosition(leftHome);
                    rightStick.setPosition(rightHome);
                } else if (gamepad1.dpad_right) {
                    leftStick.setPosition(leftOut);
                    rightStick.setPosition(rightOut);
                }
            }
        }
    }
}