package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Shooter {

    public DcMotorEx shooterMotor;
    public DcMotorEx shooterMotor2;
    private Servo magServo;
    private Servo feedServo;
    private Servo flapServo;
    private DistanceSensor ringSensor;

    public boolean magHome = true;
    public boolean feedHome = true;
    public boolean sensorBroken = true;

    public static final double SHOOTER_DX = 6.5;
    public static final double RING_SPEED = 150;
    public static final double RING_FLIGHT_TIME = 0.5;
    public static final double INITIAL_ANGLE = 0.08;

    public static double p1 = 50;
    public static double f1 = 0;
    public static double p2 = 7;
    public static double f2 = 1.15;

    private static final double RADIANS_PER_TICK = 1;
    private double targetVelocity = 0;
    private double targetPosition = 0;
    private int lock = 0; // 0: none, 1: high goal, 2: power shot
    private int numRings = 3;

    public Shooter(LinearOpMode op) {
        shooterMotor = op.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = op.hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flapServo = op.hardwareMap.get(Servo.class, "flapServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");
        ringSensor = op.hardwareMap.get(DistanceSensor.class, "ringSensor");

        flapDown();
        feedHome();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void updateShooter() {
        shooterMotor.setVelocityPIDFCoefficients(p1, 0, 0, f1);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p2, 0, 0, f2));
    }

    public void flywheelHG() {
        setFlywheelVelocity(Constants.HIGH_GOAL_VELOCITY);
    }

    public void flywheelPS() {
        setFlywheelVelocity(Constants.POWERSHOT_VELOCITY);
    }

    public void flywheelOff() {
        setFlywheelVelocity(0);
    }

    public void setFlywheelVelocity(double velocity) {
        if (velocity != targetVelocity) {
            shooterMotor.setVelocity(velocity);
            targetVelocity = velocity;
        }
    }

    public double getFlywheelVelocity() {
        return shooterMotor.getVelocity();
    }

    public void setTargetPosition(int position) {
        if (position != targetPosition) {
            shooterMotor2.setTargetPosition((int) (targetPosition / RADIANS_PER_TICK));
            targetPosition = position;
        }
    }

    public double getTargetPosition() {
        return shooterMotor2.getTargetPosition() * RADIANS_PER_TICK;
    }

    public double getVelocity() {
        return shooterMotor2.getVelocity();
    }

    public void setLock(int lockMode) {
        lock = lockMode;
    }

    public void targetLock(double x, double y, double theta, int lockMode) {
        setLock(lockMode);
        targetLock(x, y, theta);
    }

    public void targetLock(double x, double y, double theta) {
        // get angle that robot needs
        // compare with robot angle
        // determine if mod is necessary
    }

    public double getTargetVelocity() {
        return targetVelocity;
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

    public void setFlapPosition(double position) {
        flapServo.setPosition(position);
    }

    public double getFlapPosition() {
        return flapServo.getPosition();
    }

    public void flapDown() {
        flapServo.setPosition(Constants.FLAP_DOWN_POS);
    }

    public void flapUp() {
        flapServo.setPosition(Constants.FLAP_UP_POS);
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
