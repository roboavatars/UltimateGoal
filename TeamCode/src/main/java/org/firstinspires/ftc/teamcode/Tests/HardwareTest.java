package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class HardwareTest extends LinearOpMode {
    public static String motorName1 = "intake";
    public static String motorName2 = "transfer";
    public static String servoName1 = "leftClamp";
    public static String servoName2 = "rightClamp";

    public static double motor1Power = 0;
    public static double motor2Power = 0;
    public static double servo1Home = 0.95;
    public static double servo1Out = 0.4;
    public static double servo2Home = 0.05;
    public static double servo2Out = 0.6;

    public static boolean home = true;

    @Override
    public void runOpMode() {
        DcMotor motor1 = hardwareMap.get(DcMotor.class, motorName1);
        DcMotor motor2 = hardwareMap.get(DcMotor.class, motorName2);
        Servo servo1 = hardwareMap.get(Servo.class, servoName1);
        Servo servo2 = hardwareMap.get(Servo.class, servoName2);

        waitForStart();

        while(opModeIsActive()) {
            if (home) {
                servo1.setPosition(servo1Home);
                servo2.setPosition(servo2Home);
            } else {
                servo1.setPosition(servo1Out);
                servo2.setPosition(servo2Out);
            }

            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
        }
    }
}