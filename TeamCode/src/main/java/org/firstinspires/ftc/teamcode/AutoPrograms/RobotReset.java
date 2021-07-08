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
//        Servo magServo = hardwareMap.get(Servo.class, "magServo");
        Servo stackServo = hardwareMap.get(Servo.class, "stackServo");

        waitForStart();

//        magServo.setPosition(Constants.MAG_SHOOT_POS);
        stackServo.setPosition(Constants.STACK_HOME_POS);
        sleep(750);

        feedServo.setPosition(Constants.FEED_HOME_POS);
        blockerServo.setPosition(Constants.BLOCKER_HOME_POS);
        sleep(750);

        // magServo.setPosition(Constants.MAG_HOME_POS);
        sleep(500);
    }
}