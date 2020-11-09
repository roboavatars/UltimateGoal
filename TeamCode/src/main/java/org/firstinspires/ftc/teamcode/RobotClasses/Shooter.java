package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Shooter {

    private DcMotorEx shooterMotor;
    private Servo angleServo;
    private Servo magServo;
    private Servo feedServo;

    public static double angleHomePos;
    public static double magHomePos;
    public static double magShootPos;
    public static double feedHomePos;
    public static double feedShootPos;

    public boolean magHome = true;
    public boolean feedHome = true;

    public Shooter(LinearOpMode op) {
        shooterMotor = op.hardwareMap.get(DcMotorEx.class, "shooter");
        angleServo = op.hardwareMap.get(Servo.class, "angleServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");

        angleHome();
        feedHome();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void flywheelOn() {
        shooterMotor.setPower(1);
    }

    public void flywheelOff() {
        shooterMotor.setPower(0);
    }

    public void setVelocity(double velocity) {
        shooterMotor.setVelocity(velocity);
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public void setAngle(double angle) {
        // angle math here

    }

    public void angleHome() {
        angleServo.setPosition(angleHomePos);
    }

    public void magHome() {
        magServo.setPosition(magHomePos);
        magHome = true;
    }

    public void magShoot() {
        magServo.setPosition(magShootPos);
        magHome = false;
    }

    public void feedHome() {
        feedServo.setPosition(feedHomePos);
        feedHome = true;
    }

    public void feedShoot() {
        feedServo.setPosition(feedShootPos);
        feedHome = false;
    }

}
