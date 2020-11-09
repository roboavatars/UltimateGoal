package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class Test extends LinearOpMode {

    private Servo servo;
    public static double homePos = 0;
    public static double outPos = 1;
    public static boolean home = true;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "Servo");
        servo.setPosition(homePos);

        waitForStart();

        while(opModeIsActive()) {

            if (home) {
                servo.setPosition(homePos);
            } else {
                servo.setPosition(outPos);
            }
        }
    }
}
