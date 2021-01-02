package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Shooter {

    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public Servo flapServo;
    private Servo magServo;
    private Servo feedServo;

    public final int highGoalV = 1150;
    public final int powershotV = 875;

    private final double flapHomePos = 0;
    private final double flapMaxPos = 0.15;

    private final double magHomePos = 0.3;
    private final double magVibratePos = 0.27;
    private final double magShootPos = 0.495;

    private final double feedHomePos = 0.15;
    private final double feedMid = 0.2;
    private final double feedShootPos = 0.37;

    public boolean magHome = true;
    public boolean magVibrate = false;
    public boolean feedHome = true;

    public Shooter(LinearOpMode op) {
        shooterMotor1 = op.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = op.hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setVelocityPIDFCoefficients(57, 0, 0, 17);
        shooterMotor2.setVelocityPIDFCoefficients(57, 0, 0, 17);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        flapServo = op.hardwareMap.get(Servo.class, "flapServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");

        flapHome();
        feedHome();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void flywheelHighGoal() {
        setVelocity(highGoalV);
    }

    public void flywheelPowershot() {
        setVelocity(powershotV);
    }

    public void flywheelOff() {
        setVelocity(0);
    }

    public void setVelocity(double velocity) {
        shooterMotor1.setVelocity(velocity);
        shooterMotor2.setVelocity(velocity);
    }

    public double getVelocity() {
        return shooterMotor1.getVelocity();
    }

    public void setFlapAngle(double angle) {
        if (getFlapAngle() > 0 && getFlapAngle() < flapMaxPos) {
            flapServo.setPosition(angle);
        }
    }

    public double getFlapAngle() {
        return flapServo.getPosition();
    }

    public void flapHome() {
        flapServo.setPosition(flapHomePos);
    }

    public void magHome() {
        magServo.setPosition(magHomePos);
        magHome = true;
        magVibrate = false;
    }

    public void magVibrate() {
        magServo.setPosition(magVibratePos);
        magHome = false;
        magVibrate = true;
    }

    public void magShoot() {
        magServo.setPosition(magShootPos);
        magHome = false;
    }

    public void feedHome() {
        feedServo.setPosition(feedHomePos);
        feedHome = true;
    }

    public void feedMid() {
        feedServo.setPosition(feedMid);
        feedHome = true;
    }

    public void feedShoot() {
        feedServo.setPosition(feedShootPos);
        feedHome = false;
    }
}
