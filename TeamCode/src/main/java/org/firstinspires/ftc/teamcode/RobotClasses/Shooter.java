package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.*;

@SuppressWarnings("FieldCanBeLocal") @Config
public class Shooter {

    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public Servo flapServo;
    private Servo magServo;
    private Servo feedServo;
    private DistanceSensor ringSensor;

    private double lastVelocity = 0;
    private double lastFlap = 0;

    public boolean magHome = true;
    public boolean feedHome = true;

    public static double zeroDist = 4.65;
    public static double oneDist = 4.0;
    public static double twoDist = 3.3;
    public static double threeDist = 2.5;

//    private ArrayList<Integer> caching = new ArrayList(Arrays.asList(new Integer[] {-1, -1, -1, -1, -1, -1}, new Integer[0]));
//    private int cacheIndex = 0;
    private int numRings = 3;

    public Shooter(LinearOpMode op) {
        shooterMotor1 = op.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = op.hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterMotor1.setVelocityPIDFCoefficients(55, 0, 0, 16);
//        shooterMotor2.setVelocityPIDFCoefficients(55, 0, 0, 16);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        flapServo = op.hardwareMap.get(Servo.class, "flapServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");
        ringSensor = op.hardwareMap.get(DistanceSensor.class, "ringSensor");

        flapHome();
        feedHome();
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
        if (velocity != lastVelocity) {
            shooterMotor1.setVelocity(velocity);
            shooterMotor2.setVelocity(velocity);
            lastVelocity = velocity;
        }
    }

    public double getVelocity() {
        return shooterMotor1.getVelocity();
    }

    public void setFlapPosition(double position) {
        if (position != lastFlap) {
            flapServo.setPosition(position);
            lastFlap = position;
        }
    }

    public double getFlapPosition() {
        return flapServo.getPosition();
    }

    public void flapHome() {
        flapServo.setPosition(Constants.FLAP_HOME_POS);
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

    public void feedMid() {
        feedServo.setPosition(Constants.FEED_MID_POS);
        feedHome = false;
    }

    public void feedTop() {
        feedServo.setPosition(Constants.FEED_TOP_POS);
        feedHome = false;
    }

    public double getDistance() {
        double distance = ringSensor.getDistance(DistanceUnit.INCH);
        if (distance > 6) {
            Log.w("robot-log", "Ring Sensor Value Too High");
        }
        return distance;
    }

    public int getNumRings() {
        double dist = getDistance();
        int tempRings;
        if (dist > zeroDist) {
            tempRings = 0;
        } else if (dist > oneDist) {
            tempRings = 1;
        } else if (dist > twoDist) {
            tempRings = 2;
        } else {
            tempRings = 3;
        }

        if (tempRings != 3 || numRings >= 2) {
            numRings = tempRings;
        }

//        cacheIndex = (cacheIndex + 1) % caching.size();
//        caching.set(cacheIndex, numRings);
//
//        int zero = Collections.frequency(caching, 0);
//        int one = Collections.frequency(caching, 1);
//        int two = Collections.frequency(caching, 2);
//        int three = Collections.frequency(caching, 2);
//        int max = Math.max(Math.max(Math.max(zero, one), two), three);
//
//        if (zero == max) {
//            numRings = 0;
//        } else if (one == max) {
//            numRings = 1;
//        } else if (two == max) {
//            numRings = 2;
//        } else if (three == max) {
//            numRings = 3;
//        }

        return numRings;
    }
}
