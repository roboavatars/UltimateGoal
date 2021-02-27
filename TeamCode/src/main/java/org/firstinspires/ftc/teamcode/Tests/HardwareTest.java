package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
//@Disabled
public class HardwareTest extends LinearOpMode {

    public static double clampPos = 0.25;
    public static double unClampPos = 0.85;
    public static boolean clamp = true;
    public static boolean debug = true;
    private double servoPos;

    @Override
    public void runOpMode() {
        Servo wobbleServo = hardwareMap.get(Servo.class, "rightStick");

        waitForStart();

        while(opModeIsActive()) {
            if (debug) {
                if (clamp) {
                    servoPos = clampPos;
                } else {
                    servoPos = unClampPos;
                }
            } else {
                if (gamepad1.a) {
                    servoPos = clampPos;
                } else if (gamepad1.b) {
                    servoPos = unClampPos;
                }
            }
            wobbleServo.setPosition(servoPos);
        }
    }
}