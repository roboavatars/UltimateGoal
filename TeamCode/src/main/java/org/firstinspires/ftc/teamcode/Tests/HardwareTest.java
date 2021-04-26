package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@TeleOp
@Config
public class HardwareTest extends LinearOpMode {

    public static double leftHome = 0.73;
    public static double leftOut = 0.18;
    public static double rightHome = 0.55;
    public static double rightOut = 0;
    public static double stackHome = Constants.STACK_HOME_POS;
    public static double stackOut = Constants.STACK_OUT_POS;
    public static boolean home = true;
    public static boolean debug = true;

    @Override
    public void runOpMode() {
        Servo leftStick = hardwareMap.get(Servo.class, "wobbleServo");
        Servo rightStick = hardwareMap.get(Servo.class, "clampServo");
        Servo stackServo = hardwareMap.get(Servo.class, "stackServo");

        waitForStart();

        while(opModeIsActive()) {

            if (debug) {
                if (home) {
                    leftStick.setPosition(leftHome);
                    rightStick.setPosition(rightHome);
                    stackServo.setPosition(stackHome);
                } else {
                    leftStick.setPosition(leftOut);
                    rightStick.setPosition(rightOut);
                    stackServo.setPosition(stackOut);
                }
//            } else {
//                if (gamepad1.dpad_left) {
//                    leftStick.setPosition(leftHome);
//                    rightStick.setPosition(rightHome);
//                } else if (gamepad1.dpad_right) {
//                    leftStick.setPosition(leftOut);
//                    rightStick.setPosition(rightOut);
//                }
            }
        }
    }
}