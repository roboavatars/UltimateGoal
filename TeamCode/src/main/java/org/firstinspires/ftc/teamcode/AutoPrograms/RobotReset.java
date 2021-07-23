package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@Autonomous(name = "2 Robot Reset")
public class RobotReset extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo feedServo = hardwareMap.get(Servo.class, "feedServo");
        Servo blockerServo = hardwareMap.get(Servo.class, "blocker");
        Servo bumperLR = hardwareMap.get(Servo.class, "bumpers");
        Servo wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        waitForStart();

        feedServo.setPosition(Constants.FEED_HOME_POS);
        blockerServo.setPosition(Constants.BLOCKER_HOME_POS);
        wobbleServo.setPosition(Constants.WOBBLE_UP_POS);
        sleep(1000);

        bumperLR.setPosition(Constants.BUMPER_HOME_POS);
        sleep(2000);
    }
}