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

    public static double angleHome;
    public static double magHome;
    public static double magShoot;
    public static double feedHome;
    public static double feedShoot;

    public Shooter(LinearOpMode op) {
        shooterMotor = op.hardwareMap.get(DcMotorEx.class, "shooter");
        angleServo = op.hardwareMap.get(Servo.class, "angleServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");

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
        angleServo.setPosition(angleHome);
    }

    public void magHome() {
        magServo.setPosition(magHome);
    }

    public void magShoot() {
        magServo.setPosition(magShoot);
    }

    public void feedHome() {
        feedServo.setPosition(feedHome);
    }

    public void feedShoot() {
        feedServo.setPosition(feedShoot);
    }

}
