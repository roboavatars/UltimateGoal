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
    // private LynxModule module;

    // OpMode
    private LinearOpMode opMode;

    // Tracking X/Y/Theta
    public double x;
    public double y;
    public double theta;
    private double deltaheading = 0;

    // Odometry
    public double pod1 = 0;
    public double pod2 = 0;
    public double pod3 = 0;
    private double lastpod1 = 0;
    private double lastpod2 = 0;
    private double lastpod3 = 0;
    private double deltapod1;
    private double deltapod2;
    private double deltapod3;

    // Motor Caching
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;
    private final double motorUpdateTolerance = 0.05;

    // Odometry constants
    public static double ticksToInch1 = 0.00600112521;
    public static double ticksToInch2 = 0.00600112521;
    public static double ticksToInch3 = 0.00600112521;
    public static double OdometryTrackWidth = 13.655;
    public static double OdometryHorizontalOffset = -1.83;
    private final double OdometryHeadingThreshold = PI/8;

    // PD controller constants
    public final static double xKp = 0.7;
    public final static double yKp = 0.6;
    public final static double thetaKp = 5.0;
    public final static double xKd = 0.05;
    public final static double yKd = 0.05;
    public final static double thetaKd = 0.2;

    // Odometry delta 0 counters
    public int zero1, zero2, zero3;

    public boolean zeroStrafeCorrection = false;

    // Constructor
    public MecanumDrivetrain(LinearOpMode opMode, double initialX, double initialY, double initialTheta) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        // module = hardwareMap.get(LynxModule.class, "Control Hub");

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
        theta = initialTheta;
    }

    // reset odometry
    public void resetOdo(double newX, double newY, double newTheta) {
        x = newX;
        y = newY;
        theta = newTheta;
    }

    // robot centric movement
    public void setControls(double xdot, double ydot, double w) {
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

    // update position from odometry
    public void updatePose() {
        try {
            pod1 = motorFrontLeft.getCurrentPosition() * ticksToInch1;
            pod2 = motorBackLeft.getCurrentPosition() * -ticksToInch2;
            pod3 = motorFrontRight.getCurrentPosition() * ticksToInch3;

            deltapod1 = pod1 - lastpod1;
            deltapod2 = pod2 - lastpod2;
            deltapod3 = pod3 - lastpod3;

            if (!(deltapod1 == 0 && deltapod2 == 0 && deltapod3 == 0)) {
                if (deltapod1 == 0) {
                    Log.w("pod-delta-log", "pod1 delta 0");
                    zero1++;
                }
                if (deltapod2 == 0) {
                    Log.w("pod-delta-log", "pod2 delta 0");
                    zero2++;
                }
                if (deltapod3 == 0) {
                    Log.w("pod-delta-log", "pod3 delta 0");
                    zero3++;
                }
            }

            deltaheading = (deltapod2 - deltapod1) / OdometryTrackWidth;

            double localX = (deltapod1 + deltapod2) / 2;
            double localY = deltapod3 - deltaheading * OdometryHorizontalOffset;

            if (deltaheading < OdometryHeadingThreshold) {
                x += localX * Math.cos(theta) - localY * Math.sin(theta);
                y += localY * Math.cos(theta) + localX * Math.sin(theta);

            } else {
                x += (localX * Math.sin(theta + deltaheading) + localY * Math.cos(theta + deltaheading)
                        - localX * Math.sin(theta) - localY * Math.cos(theta)) / deltaheading;
                y += (localY * Math.sin(theta + deltaheading) - localX * Math.cos(theta + deltaheading)
                        - localY * Math.sin(theta) + localX * Math.cos(theta)) / deltaheading;
            }

            theta += deltaheading;
            theta = theta % (2*PI);
            if (theta < 0) theta += 2*PI;

            lastpod1 = pod1;
            lastpod2 = pod2;
            lastpod3 = pod3;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}