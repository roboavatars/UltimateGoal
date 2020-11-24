package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Shooter {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    public Servo flapServo;
    private Servo magServo;
    private Servo feedServo;

    private DistanceSensor ringSensor;

    private final double flapHomePos = 0;
    private final double flapMaxPos = 0.25;
    private final double magHomePos = 0.25;
    private final double magShootPos = 0.50;
    private final double feedHomePos = 0.45;
    private final double feedShootPos = 0.70;

    public boolean magHome = true;
    public boolean feedHome = true;

    public Shooter(LinearOpMode op) {
        shooterMotor1 = op.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = op.hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flapServo = op.hardwareMap.get(Servo.class, "flapServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");
        //ringSensor = op.hardwareMap.get(DistanceSensor.class, "ringSensor");

        flapHome();
        feedHome();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void flywheelOn() {
        shooterMotor1.setVelocity(-2000);
        shooterMotor2.setVelocity(-2000);
    }

    public void flywheelOff() {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
    }

    public void setShooterVelocity(double velocity) {
        shooterMotor1.setVelocity(velocity);
        shooterMotor2.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        return (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2;
    }

    public void setFlapAngle(double angle) {
        flapServo.setPosition(angle);
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

    public double getDistance() {
        return ringSensor.getDistance(DistanceUnit.CM);
    }

    public int ringsInMag() {
        /*if (getDistance() >  && getDistance() < ) {
            return 3;
        } else if (getDistance() > && getDistance() < ) {
            return 2;
        } else if (getDistance() > && getDistance() < ) {
            return 1;
        } else {
            return 0;
        }*/
        return 0;
    }

}
