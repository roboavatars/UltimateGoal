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

    private DcMotorEx motor;
    private Servo servo;
    public static double homePos = 0.3;
    public static double outPos = 1;
    public static boolean home = true;

    public static boolean home2 = true;
    private int wobbleHomePos = -100;
    private int wobbleClampPos = -750;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        servo = hardwareMap.get(Servo.class, "wobbleServo");

        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()) {

            if (home) {
                servo.setPosition(homePos);
            } else {
                servo.setPosition(outPos);
            }
//            if (gamepad1.dpad_up) {
//                servo.setPosition(servo.getPosition() + 0.01);
//            } else if (gamepad1.dpad_down) {
//                servo.setPosition(servo.getPosition() - 0.01);
//            }

            if (home2) {
                motor.setTargetPosition(wobbleHomePos);
                motor.setPower(0.75);
            } else {
                motor.setTargetPosition(wobbleClampPos);
                motor.setPower(-0.75);
            }

            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}