package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal") @Config
public class MecanumDrivetrain {

    // Electronics
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    // OpMode
    private LinearOpMode op;

    // Tracking X/Y/Theta
    public double x, y, theta, startTheta;
    private double lastRawHeading = 0;
    private double deltaHeading = 0;
    public double commandedW;

    // Odometry
    public double pod1 = 0;
    public double pod2 = 0;
    public double pod3 = 0;
    private double lastPod1 = 0;
    private double lastPod2 = 0;
    private double lastPod3 = 0;
    private double deltaPod1;
    private double deltaPod2;
    private double deltaPod3;

    // Motor Caching
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;
    private final double motorUpdateTolerance = 0.05;

    // Odometry constants
    public static double ticksToInch1 = 0.0005268504;
    public static double ticksToInch2 = 0.0005268504;
    public static double ticksToInch3 = 0.0005292873;
    public static double ODOMETRY_TRACK_WIDTH = 13.28;
    //       1 rev                  5 rev
    // cw:  13.28                  13.247 13.232
    // ccw: 13.335 13.32 13.31

    //                          13.39 5 rev         13.39 10 rev                13.395 5 rev          1.385 5 rev
    // cw:  1.56 1.56 1.57 | 1.59 1.59 1.58 1.58 | 1.59 1.58 1.53 1.55 1.56 | 1.58 1.58 1.58 1.58 | 1.55 1.54
    // ccw: 1.63 1.63 1.62 | 1.60 1.61 1.60 1.60 | 1.66 1.69 1.67 1.68 1.66 | 1.63 1.61 1.62 1.60 | 1.62 1.63
    public static double ODOMETRY_HORIZONTAL_OFFSET = 0.15;

    private final double ODOMETRY_HEADING_THRESHOLD = PI/8;

    // PD controller constants
    public final static double xKp = 0.6;
    public final static double yKp = 0.55;
    public final static double thetaKp = 3.0;
    public final static double xKd = 0.05;
    public final static double yKd = 0.05;
    public final static double thetaKd = 0.07;

    // Odometry delta 0 counters
    public int zero1, zero2, zero3;

    public boolean zeroStrafeCorrection = false;

    public double lastHeading;

//    private IMU imu;
    private T265 t265;

    // Constructor
    public MecanumDrivetrain(LinearOpMode op, double initialX, double initialY, double initialTheta) {
        this.op = op;
        HardwareMap hardwareMap = op.hardwareMap;

//        imu = new IMU(initialTheta, op);
        t265 = new T265(op, initialX, initialY, initialTheta);
        t265.startCam();

        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        x = initialX;
        y = initialY;
        startTheta = initialTheta;
        theta = initialTheta;
        lastHeading = theta;
    }

    // reset odometry
    public void resetOdo(double newX, double newY, double newTheta) {
        x = newX;
        y = newY;
        t265.resetTheta(newTheta);
//        imu.resetHeading(newTheta);
    }

    // robot centric movement
    public void setControls(double xdot, double ydot, double w) {
        commandedW = w;

        double FRpower, FLpower, BRpower, BLpower;

        if (!zeroStrafeCorrection) {
            FRpower = ydot + xdot + w;
            FLpower = -ydot + xdot - w;
            BRpower = -ydot + xdot + w;
            BLpower = ydot + xdot - w;
        } else {
            FRpower = xdot + w;
            FLpower = xdot - w;
            BRpower = xdot + w;
            BLpower = xdot - w;
        }

        double maxpower = Math.max(Math.abs(FRpower), Math.max(Math.abs(FLpower), Math.max(Math.abs(BRpower), Math.abs(BLpower))));

        if (maxpower > 1) {
            FRpower /= maxpower;
            FLpower /= maxpower;
            BRpower /= maxpower;
            BLpower /= maxpower;
        }

        if (xdot == 0 && ydot == 0 && w == 0) {
            // Set Motor Powers
            motorFrontRight.setPower(FRpower);
            motorFrontLeft.setPower(FLpower);
            motorBackRight.setPower(BRpower);
            motorBackLeft.setPower(BLpower);

            // Cache New Motor Powers
            lastFRPower = FRpower;
            lastFLPower = FLpower;
            lastBRPower = BRpower;
            lastBLPower = BLpower;

        } else if (Math.abs(FRpower - lastFRPower) > motorUpdateTolerance || Math.abs(FLpower - lastFLPower) > motorUpdateTolerance
                || Math.abs(BRpower - lastBRPower) > motorUpdateTolerance || Math.abs(BLpower - lastBLPower) > motorUpdateTolerance) {

            // Set Motor Powers
            motorFrontRight.setPower(FRpower);
            motorFrontLeft.setPower(FLpower);
            motorBackRight.setPower(BRpower);
            motorBackLeft.setPower(BLpower);

            // Cache New Motor Powers
            lastFRPower = FRpower;
            lastFLPower = FLpower;
            lastBRPower = BRpower;
            lastBLPower = BLpower;
        }
    }

    public void setRawPower(double frontRight, double frontLeft, double backRight, double backLeft) {
        // Set Motor Powers
        motorFrontRight.setPower(frontRight);
        motorFrontLeft.setPower(frontLeft);
        motorBackRight.setPower(backRight);
        motorBackLeft.setPower(backLeft);

        // Cache New Motor Powers
        lastFRPower = frontRight;
        lastFLPower = frontLeft;
        lastBRPower = backRight;
        lastBLPower = backLeft;
    }

    // field centric movement
    public void setGlobalControls(double xvelocity, double yvelocity, double w) {
        double xdot = xvelocity * Math.cos(-theta) - yvelocity * Math.sin(-theta);
        double ydot = yvelocity * Math.cos(-theta) + xvelocity * Math.sin(-theta);
        setControls(xdot, ydot, w);
    }

    // stop drivetrain
    public void stop() {
        setGlobalControls(0, 0, 0);
    }

    public double getRawTheta() {
        return t265.getRawTheta();
    }

    public double getThetaError() {
        return t265.getThetaError();
    }

    public double getInitTheta() {
        return t265.getInitTheta();
    }

    public void updateThetaError() {
        t265.makeSureT265IsGood();
    }

    // update position from odometry
    public void updatePose() {
        try {
            pod1 = motorBackRight.getCurrentPosition() * ticksToInch1;
            pod2 = motorFrontLeft.getCurrentPosition() * -ticksToInch2;
            pod3 = motorFrontRight.getCurrentPosition() * -ticksToInch3;

            deltaPod1 = pod1 - lastPod1;
            deltaPod2 = pod2 - lastPod2;
            deltaPod3 = pod3 - lastPod3;

            t265.updateCamPose();

            theta = t265.getTheta() % (2*PI);
            if (theta < 0) {
                theta += 2*PI;
            }
            deltaHeading = theta - lastHeading;

            if (Math.abs(t265.getTheta() - lastRawHeading) > PI/4) {
                t265.thetaError += deltaHeading;
                Robot.log("T265 Disaster Averted " + t265.getTheta() + " " + lastRawHeading + " " + deltaHeading + " " + t265.thetaError);
            }
            lastRawHeading = t265.getTheta();
//            deltaPod1 = deltaPod2 - deltaHeading * ODOMETRY_TRACK_WIDTH;

//            imu.updateHeading();
//            theta = imu.getTheta() % (2*PI);
//            deltaHeading = imu.getDeltaHeading();
//            deltaPod1 = deltaPod2 - deltaHeading * ODOMETRY_TRACK_WIDTH;

            if (!(deltaPod1 == 0 && deltaPod2 == 0 && deltaPod3 == 0)) {
                if (deltaPod1 == 0) {
                    Log.w("pod-delta-log", "pod1 delta 0");
                    zero1++;
                }
                if (deltaPod2 == 0) {
                    Log.w("pod-delta-log", "pod2 delta 0");
                    zero2++;
                }
                if (deltaPod3 == 0) {
                    Log.w("pod-delta-log", "pod3 delta 0");
                    zero3++;
                }
            }

//            deltaHeading = (deltaPod2 - deltaPod1) / ODOMETRY_TRACK_WIDTH;

            double localX = (deltaPod1 + deltaPod2) / 2;
            double localY = deltaPod3 - deltaHeading * ODOMETRY_HORIZONTAL_OFFSET;

//            Robot.log(deltaPod1 + " " + deltaPod2 + " " + deltaPod3 + " " + deltaHeading);

            if (deltaHeading < ODOMETRY_HEADING_THRESHOLD) {
                x += localX * Math.cos(theta) - localY * Math.sin(theta);
                y += localY * Math.cos(theta) + localX * Math.sin(theta);

            } else {
                x += (localX * Math.sin(theta + deltaHeading) + localY * Math.cos(theta + deltaHeading)
                        - localX * Math.sin(theta) - localY * Math.cos(theta)) / deltaHeading;
                y += (localY * Math.sin(theta + deltaHeading) - localX * Math.cos(theta + deltaHeading)
                        - localY * Math.sin(theta) + localX * Math.cos(theta)) / deltaHeading;
            }

//            theta = startTheta + (pod2 - pod1) / ODOMETRY_TRACK_WIDTH;
//            theta = theta % (2*PI);
//            if (theta < 0) theta += 2*PI;

            lastPod1 = pod1;
            lastPod2 = pod2;
            lastPod3 = pod3;
            lastHeading = theta;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}