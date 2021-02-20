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

        magServo.setPosition(Constants.MAG_SHOOT_POS);
        sleep(1000);
        feedServo.setPosition(Constants.FEED_TOP_POS);
        sleep(500);
        magServo.setPosition(Constants.MAG_HOME_POS);
    }
}