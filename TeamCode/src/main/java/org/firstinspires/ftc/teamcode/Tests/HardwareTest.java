package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

@TeleOp
@Config
public class HardwareTest extends LinearOpMode {

    public static double leftHome = 1;
    public static double leftOut = 0;
    public static double rightHome = 0;
    public static double rightOut = 1;
    public static boolean home = true;
    public static boolean debug = true;

    @Override
    public void runOpMode() {
        Servo leftStick = hardwareMap.get(Servo.class, "leftStick");
        Servo rightStick = hardwareMap.get(Servo.class, "rightStick");

//        Servo standoff = hardwareMap.get(Servo.class, "standoff");
        Intake intake = new Intake(this, false);

        Servo magServo = hardwareMap.get(Servo.class, "magServo");

        waitForStart();

        while(opModeIsActive()) {
            if (debug) {
                if (home) {
                    leftStick.setPosition(leftHome);
                    rightStick.setPosition(rightHome);

//                    standoff.setPosition(leftHome);
                } else {
                    leftStick.setPosition(leftOut);
                    rightStick.setPosition(rightOut);

//                    standoff.setPosition(leftOut);
                }
            }

//            else {
//                if (gamepad1.dpad_left) {
//                    leftStick.setPosition(leftHome);
//                    rightStick.setPosition(rightHome);
//                } else if (gamepad1.dpad_right) {
//                    leftStick.setPosition(leftOut);
//                    rightStick.setPosition(rightOut);
//                }
//            }

            if (gamepad1.x) {
                magServo.setPosition(Constants.MAG_SHOOT_POS);
            } else {
                magServo.setPosition(Constants.MAG_HOME_POS);
            }

            if (gamepad1.right_trigger > 0) {
                intake.on();
            } else if (gamepad1.left_trigger > 0) {
                intake.reverse();
            } else {
                intake.off();
            }
        }
    }
}