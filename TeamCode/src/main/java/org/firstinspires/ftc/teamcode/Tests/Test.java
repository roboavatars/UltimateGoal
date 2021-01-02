package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp @Config
public class Test extends LinearOpMode {

    public static double lHomePos = 0.95;
    public static double lOutPos = 0.05;
    public static double rHomePos = 0.25;
    public static double rOutPos = 0.9;
    public static boolean isLHome = true;
    public static boolean isRHome = true;
    private double lPosition;
    private double rPosition;

    @Override
    public void runOpMode() {

        Servo leftServo = hardwareMap.get(Servo.class, "leftStick");
        Servo rightServo = hardwareMap.get(Servo.class, "rightStick");

        waitForStart();

        while(opModeIsActive()) {
            if (isLHome) {
                lPosition = lHomePos;
            } else {
                lPosition = lOutPos;
            }

            leftServo.setPosition(lPosition);

            if (isRHome) {
                rPosition = rHomePos;
            } else {
                rPosition = rOutPos;
            }

            rightServo.setPosition(rPosition);
        }
    }
}