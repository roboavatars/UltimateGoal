package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@TeleOp
@Config
//@Disabled
public class HardwareTest extends LinearOpMode {

    public static double clampPos = 0.55;
    public static double unClampPos = 0;
    public static boolean clamp = true;
    public static boolean debug = false;
    private double servoPos;

    @Override
    public void runOpMode() {
        Servo clampServo = hardwareMap.get(Servo.class, "clampServo");
        Servo armServo = hardwareMap.get(Servo.class, "wobbleServo");

        waitForStart();

        while(opModeIsActive()) {
            if (debug) {
                if (clamp) {
                    servoPos = clampPos;
                } else {
                    servoPos = unClampPos;
                }
            } else {
                if (gamepad1.dpad_left) {
                    clampServo.setPosition(Constants.WOBBLE_UNCLAMP_POS);
                } else if (gamepad1.dpad_right) {
                    clampServo.setPosition(Constants.WOBBLE_CLAMP_POS);
                } else if (gamepad1.dpad_down) {
                    armServo.setPosition(Constants.WOBBLE_DOWN_POS);
                } else if (gamepad1.dpad_up) {
                    armServo.setPosition(Constants.WOBBLE_UP_POS);
                }
            }
        }
    }
}