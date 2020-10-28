package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Shooter {

    private DcMotorEx shooterMotor;
    private Servo angleServo;

    public static double OPEN_ANGLE = 0;
    public static double CLOSE_ANGLE = 1;

    public Shooter(LinearOpMode op) {
        shooterMotor = op.hardwareMap.get(DcMotorEx.class, "shooter");
        angleServo = op.hardwareMap.get(Servo.class, "angle");

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

    public void open() {
        angleServo.setPosition(OPEN_ANGLE);
    }

    public void close() {
        angleServo.setPosition(CLOSE_ANGLE);
    }

}
