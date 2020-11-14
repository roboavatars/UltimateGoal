package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class Test extends LinearOpMode {

    private Servo servo;
    public static double homePos = 0;
    public static double outPos = 0.5;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "magServo");
        servo.setPosition(homePos);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servo.setPosition(servo.getPosition() + 0.01);
            }
            if (gamepad1.dpad_down) {
                servo.setPosition(servo.getPosition() - 0.01);
            }
            telemetry.addData("Value", servo.getPosition());
            telemetry.update();
        }
    }
}
