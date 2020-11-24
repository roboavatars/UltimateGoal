package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Config
public class Test extends LinearOpMode {

    private Servo servo;
    private Servo servo2;
    public static double homePos = 0.92;
    public static double homePos2 = 0;
    public static double outPos = 0;
    public static double outPos2 = 0.85;
    public static boolean home = true;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "leftStick");
        servo2 = hardwareMap.get(Servo.class, "rightStick");

        waitForStart();

        while(opModeIsActive()) {

            if (home) {
                servo.setPosition(homePos);
                servo2.setPosition(homePos2);
            } else {
                servo.setPosition(outPos);
                servo2.setPosition(outPos2);
            }

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}