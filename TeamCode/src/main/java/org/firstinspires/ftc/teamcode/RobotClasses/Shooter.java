package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Shooter {

    public DcMotorEx flywheelMotor;
    public DcMotorEx turretMotor;
    private Servo magServo;
    private Servo feedServo;
    private Servo flapServo;

    public boolean magHome = true;
    public boolean feedHome = true;

    public static final double TURRET_DX = 0.5;
    public static final double TURRET_DY = -4;
    public static final double TURRET_DIAMETER = 8.5;
    public static final double RING_SPEED = 150;
    public static final double RING_FLIGHT_TIME = 0.5;
    public static final double INITIAL_ANGLE = 0.08;

    public static double pFlywheel = 80;
    public static double dFlywheel = 0;
    public static double fFlywheel = 13;

    public static double pTurret = 2.25;
    public static double dTurret = 5.5;
    public static double fTurret = 0;
    public double initialTheta;

    private double targetTheta = 0;
    private double turretTheta;
    private double turretError;
    private double turretErrorChange;
    private double lockTheta;

    private static final double TICKS_PER_RADIAN = 466.2 / PI;
    private double targetVelocity = 0;

    private int count = 0;
    private final int currentCheckInterval = 2000;
    private boolean stalling = false;

    public Shooter(LinearOpMode op, double initialTheta) {
        this.initialTheta = initialTheta;

        flywheelMotor = op.hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setVelocityPIDFCoefficients(pFlywheel, 0, dFlywheel, fFlywheel);

        turretMotor = op.hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flapServo = op.hardwareMap.get(Servo.class, "flapServo");
        magServo = op.hardwareMap.get(Servo.class, "magServo");
        feedServo = op.hardwareMap.get(Servo.class, "feedServo");

        flapDown();
        feedHome();
        magHome();

        op.telemetry.addData("Status", "Shooter initialized");
    }

    public void updatePID(double robotTheta) {
        if (lockTheta == Double.MAX_VALUE) {
            turretMotor.setPower(0);
            return;
        }
        targetTheta = (lockTheta - robotTheta + PI/2) % (2*PI);
        if (targetTheta < 0) {
            targetTheta += 2*PI;
        }
        if (targetTheta > 3*PI/2) {
            targetTheta -= 2*PI;
        }
        targetTheta = Math.min(Math.max(targetTheta, -PI/4), 7*PI/6);
        turretTheta = getTheta();
        turretErrorChange = targetTheta - turretTheta - turretError;
        turretError = targetTheta - turretTheta;

        if (count % currentCheckInterval == 0) {
            if (turretMotor.getCurrent(CurrentUnit.MILLIAMPS) > 9000) {
                stalling = true;
                turretMotor.setPower(0);
                Robot.log("Turret motor stall detected!");
            } else {
                stalling = false;
            }
        } else if (!stalling) {
            turretMotor.setPower(pTurret * turretError + dTurret * turretErrorChange + fTurret);
        }

        if (stalling) {
            Robot.log("Turret motor stalling!!!");
        }
        count++;
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
        lockTheta = theta;
    }

    public double getTargetTheta() {
        return targetTheta;
    }

    public double getTheta() {
        return turretMotor.getCurrentPosition() / TICKS_PER_RADIAN + initialTheta;
    }

    public double getTurretVelocity() {
        return turretMotor.getVelocity();
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

    public void feedShoot() {
        feedServo.setPosition(Constants.FEED_SHOOT_POS);
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
}
