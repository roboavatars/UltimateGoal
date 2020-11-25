package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class Test extends LinearOpMode {

    private Servo servo;
    public static double homePos = 0.17;
    public static double outPos = 0.75;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "wobbleServo");

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.a) {
                servo.setPosition(homePos);
            } else if (gamepad1.b) {
                servo.setPosition(outPos);
            }

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}