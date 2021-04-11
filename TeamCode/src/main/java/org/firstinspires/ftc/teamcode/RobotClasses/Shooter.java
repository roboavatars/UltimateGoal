package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Shooter {

    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    private Servo magServo;
    public Servo feedServo;
    private DistanceSensor ringSensor;

    public boolean magHome = true;
    public boolean feedHome = false;
    public boolean sensorBroken = true;

    public static final double SHOOTER_DX = 6.5;
    public static final double RING_SPEED = 150;
    public static final double INITIAL_ANGLE = 0.08;

    private double targetVelocity = 0;
    private int numRings = 3;

    public Shooter(LinearOpMode op) {
        shooterMotor1 = op.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = op.hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterMotor1.setVelocityPIDFCoefficients(54, 0, 0, 14);
//        shooterMotor2.setVelocityPIDFCoefficients(54, 0, 0, 14);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");
        ringSensor = op.hardwareMap.get(DistanceSensor.class, "ringSensor");

        feedTop();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void flywheelHG() {
        setVelocity(Constants.HIGH_GOAL_VELOCITY);
    }

    public void flywheelPS() {
        setVelocity(Constants.POWERSHOT_VELOCITY);
    }

    public void flywheelOff() {
        setVelocity(0);
    }

    public void setVelocity(double velocity) {
//        if (velocity != lastVelocity) {
            shooterMotor1.setVelocity(velocity);
            shooterMotor2.setVelocity(velocity);
            Robot.log(velocity + "");
            targetVelocity = velocity;
//        }
    }

    public double getVelocity() {
        return (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity1() {
        return shooterMotor1.getVelocity();
    }

    public double getVelocity2() {
        return shooterMotor2.getVelocity();
    }

    public void magHome() {
        magServo.setPosition(Constants.MAG_HOME_POS);
        magHome = true;
    }

    public void magShoot() {
        magServo.setPosition(Constants.MAG_SHOOT_POS);
        magHome = false;
    }

    public void feedHome() {
        feedServo.setPosition(Constants.FEED_HOME_POS);
        feedHome = true;
    }

    public void feedTop() {
        feedServo.setPosition(Constants.FEED_TOP_POS);
        feedHome = false;
    }

    public double getDistance() {
        double distance = ringSensor.getDistance(DistanceUnit.INCH);
//        sensorBroken = distance > 6;
        return distance;
    }

    public int getNumRings() {
        double dist = getDistance();
        int tempRings;
        if (dist > Constants.ZERO_DIST) {
            tempRings = 0;
        } else if (dist > Constants.ONE_DIST) {
            tempRings = 1;
        } else if (dist > Constants.TWO_DIST) {
            tempRings = 2;
        } else {
            tempRings = 3;
        }

        if (tempRings != 3 || numRings >= 2) {
            numRings = tempRings;
        }

        return numRings;
    }
}
