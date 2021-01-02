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
        setVelocity(Constants.HIGH_GOAL_VELOCITY);
    }

    public void flywheelPowershot() {
        setVelocity(Constants.POWERSHOT_VELOCITY);
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
        if (getFlapAngle() > 0 && getFlapAngle() < Constants.FLAP_MAX_POS) {
            flapServo.setPosition(angle);
        }
    }

    public double getFlapAngle() {
        return flapServo.getPosition();
    }

    public void flapHome() {
        flapServo.setPosition(Constants.FLAP_HOME_POS);
    }

    public void magHome() {
        magServo.setPosition(Constants.MAG_HOME_POS);
        magHome = true;
        magVibrate = false;
    }

    public void magVibrate() {
        magServo.setPosition(Constants.MAG_VIBRATE_POS);
        magHome = false;
        magVibrate = true;
    }

    public void magShoot() {
        magServo.setPosition(Constants.MAG_SHOOT_POS);
        magHome = false;
    }

    public void feedHome() {
        feedServo.setPosition(Constants.FEED_HOME_POS);
        feedHome = true;
    }

    public void feedMid() {
        feedServo.setPosition(Constants.FEED_MID_POS);
        feedHome = true;
    }

    public void feedShoot() {
        feedServo.setPosition(Constants.FEED_SHOOT_POS);
        feedHome = false;
    }
}
