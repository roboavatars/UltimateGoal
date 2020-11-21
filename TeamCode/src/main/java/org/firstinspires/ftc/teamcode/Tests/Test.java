package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class Test extends LinearOpMode {

    private DcMotorEx motor;
    private Servo servo;
    public static double homePos = 0;
    public static double outPos = 0.5;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        servo = hardwareMap.get(Servo.class, "wobbleServo");
        servo.setPosition(0);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servo.setPosition(servo.getPosition() + 0.01);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(servo.getPosition() - 0.01);
            }

            if (gamepad1.dpad_left) {
                motor.setTargetPosition(motor.getCurrentPosition() + 2);
            } else if (gamepad1.dpad_right) {
                motor.setTargetPosition(motor.getCurrentPosition() - 2);
            }

            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}