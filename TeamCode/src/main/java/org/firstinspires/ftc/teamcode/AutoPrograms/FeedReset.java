package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@Autonomous(name = "Feed Reset")
public class FeedReset extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo magServo = hardwareMap.get(Servo.class, "magServo");
        Servo feedServo = hardwareMap.get(Servo.class, "feedServo");

        waitForStart();

        magServo.setPosition(0.5);
        sleep(1500);
        feedServo.setPosition(Constants.FEED_TOP_POS);
        sleep(500);
        magServo.setPosition(Constants.MAG_HOME_POS);
    }
}