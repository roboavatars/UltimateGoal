package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Shooter {

    public DcMotorEx flywheelMotor;
    public DcMotorEx turretMotor;
    private Servo magServo;
    private Servo feedServo;
    private Servo flapServo;

//    private DistanceSensor ringSensor;
//    public boolean sensorBroken = true;
//    private int numRings = 3;

    public boolean magHome = true;
    public boolean feedHome = true;

    public static final double TURRET_DX = 6.5;
    public static final double TURRET_DY = -1;
    public static final double TURRET_DIAMETER = 8.5;
    public static final double RING_SPEED = 150;
    public static final double RING_FLIGHT_TIME = 0.5;
    public static final double INITIAL_ANGLE = 0.08;

    public static double pFlywheel = 50;
    public static double fFlywheel = 0;

    public static double pTurret = 0.6;
    public static double dTurret = 5.5;
    public static double fTurret = 0.04;

    private double targetTheta = 0;
    private double turretTheta;
    private double turretError;
    private double turretErrorChange;
    private double lockFactor;

    private static final double TICKS_PER_RADIAN = 126 / PI;
    private double targetVelocity = 0;

    public Shooter(LinearOpMode op) {
        flywheelMotor = op.hardwareMap.get(DcMotorEx.class, "flywheel");
        turretMotor = op.hardwareMap.get(DcMotorEx.class, "turret");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flapServo = op.hardwareMap.get(Servo.class, "flapServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");
//        ringSensor = op.hardwareMap.get(DistanceSensor.class, "ringSensor");

        flapDown();
        feedHome();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void updatePID(double robotTheta) {
        targetTheta = lockFactor - robotTheta;
        turretTheta = turretMotor.getCurrentPosition() / TICKS_PER_RADIAN;
        turretErrorChange = targetTheta - turretTheta - turretError;
        turretError = targetTheta - turretTheta;
        turretMotor.setPower(Math.max(-0.5, Math.min(pTurret * turretError + dTurret * turretErrorChange + fTurret, 0.5)));

        flywheelMotor.setVelocityPIDFCoefficients(pFlywheel, 0, 0, fFlywheel);
    }

    // Flywheel
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
            flywheelMotor.setVelocity(velocity);
            targetVelocity = velocity;
        }
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getFlywheelVelocity() {
        return flywheelMotor.getVelocity();
    }

    // Turret
    public void setTargetTheta(double theta) {
        if (theta != targetTheta) {
//            lockFactor
            targetTheta = theta;
        }
    }

    public double getTargetTheta() {
        return targetTheta;
    }

    public double getTheta() {
        return turretTheta;
    }

    public double getTurretVelocity() {
        return turretMotor.getVelocity() / TICKS_PER_RADIAN;
    }

    // Mag
    public void magHome() {
        magServo.setPosition(Constants.MAG_HOME_POS);
        magHome = true;
    }

    public void magShoot() {
        magServo.setPosition(Constants.MAG_SHOOT_POS);
        magHome = false;
    }

    // Flicker
    public void feedHome() {
        feedServo.setPosition(Constants.FEED_HOME_POS);
        feedHome = true;
    }

    public void feedTop() {
        feedServo.setPosition(Constants.FEED_TOP_POS);
        feedHome = false;
    }

    // Flap
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

    /*public double getDistance() {
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
    }*/
}
