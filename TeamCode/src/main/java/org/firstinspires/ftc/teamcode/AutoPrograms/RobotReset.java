package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@Autonomous(name = "Robot Reset")
public class RobotReset extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo feedServo = hardwareMap.get(Servo.class, "feedServo");
        Servo leftStick = hardwareMap.get(Servo.class, "leftStick");
        Servo rightStick = hardwareMap.get(Servo.class, "rightStick");
        Servo blockerServo = hardwareMap.get(Servo.class, "blocker");

        waitForStart();

        leftStick.setPosition(Constants.L_HALF_POS);
        rightStick.setPosition(Constants.R_HALF_POS);
        sleep(1500);
        feedServo.setPosition(Constants.FEED_HOME_POS);
        blockerServo.setPosition(Constants.BLOCKER_HOME_POS);
        sleep(1000);
        leftStick.setPosition(Constants.L_HOME_POS);
        rightStick.setPosition(Constants.R_HOME_POS);
        sleep(1000);
    }
}