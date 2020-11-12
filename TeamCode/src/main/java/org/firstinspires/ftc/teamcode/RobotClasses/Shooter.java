package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Shooter {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    public Servo angleServo;
    private Servo magServo;
    private Servo feedServo;

    private DistanceSensor ringSensor;

    public static double angleHomePos = 0;
    public static double angleMaxPos = 0.25;
    public static double magHomePos = 0.25;
    public static double magShootPos = 0.50;
    public static double feedHomePos = 0.50;
    public static double feedShootPos = 0.65;

    public boolean magHome = true;
    public boolean feedHome = true;

    public Shooter(LinearOpMode op) {
        shooterMotor1 = op.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = op.hardwareMap.get(DcMotorEx.class, "shooter2");
        angleServo = op.hardwareMap.get(Servo.class, "angleServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");
        //ringSensor = op.hardwareMap.get(DistanceSensor.class, "ringSensor");

        angleHome();
        feedHome();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void flywheelOn() {
        shooterMotor1.setPower(-1);
        shooterMotor2.setPower(-1);
    }

    public void flywheelOff() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    public void setVelocity(double velocity) {
        shooterMotor1.setVelocity(velocity);
        shooterMotor2.setVelocity(-velocity);
    }

    public double[] getVelocity() {
        return new double[] {shooterMotor1.getVelocity(), shooterMotor2.getVelocity()};
    }

    public void setFlapAngle(double angle) {
        angleServo.setPosition(angle);
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
