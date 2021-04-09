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
    public static double ticksToInch = 0.00599227948;
    public static double OdometryTrackWidth = 13.595;
    public static double OdometryHorizontalOffset = -2.875;
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
        double FRpower;
        double BLpower;
        double FLpower;
        double BRpower;

        if (!zeroStrafeCorrection) {
            FRpower = ydot + xdot + w;
            BLpower = ydot + xdot - w;
            FLpower = -ydot + xdot - w;
            BRpower = -ydot + xdot + w;
        } else {
            FRpower = xdot + w;
            BLpower = xdot - w;
            FLpower = xdot - w;
            BRpower = xdot + w;
        }

        double maxpower = Math.max(Math.abs(FRpower), Math.max(Math.abs(BLpower), Math.max(Math.abs(FLpower), Math.abs(BRpower))));

        if (maxpower > 1) {
            FRpower /= maxpower;
            BLpower /= maxpower;
            FLpower /= maxpower;
            BRpower /= maxpower;
        }

        if (xdot == 0 && ydot == 0 && w == 0) {
            // Set Motor Powers
            motorFrontRight.setPower(FRpower);
            motorBackLeft.setPower(BLpower);
            motorFrontLeft.setPower(FLpower);
            motorBackRight.setPower(BRpower);

            // Cache New Motor Powers
            lastFRPower = FRpower;
            lastFLPower = FLpower;
            lastBRPower = BRpower;
            lastBLPower = BLpower;

        } else if (Math.abs(FRpower - lastFRPower) > motorUpdateTolerance || Math.abs(FLpower - lastFLPower) > motorUpdateTolerance
                || Math.abs(BRpower - lastBRPower) > motorUpdateTolerance || Math.abs(BLpower - lastBLPower) > motorUpdateTolerance) {

            // Set Motor Powers
            motorFrontRight.setPower(FRpower);
            motorBackLeft.setPower(BLpower);
            motorFrontLeft.setPower(FLpower);
            motorBackRight.setPower(BRpower);

            // Cache New Motor Powers
            lastFRPower = FRpower;
            lastFLPower = FLpower;
            lastBRPower = BRpower;
            lastBLPower = BLpower;
        }
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
            pod1 = motorFrontRight.getCurrentPosition() * -ticksToInch;
            pod2 = motorBackRight.getCurrentPosition() * ticksToInch;
            pod3 = motorFrontLeft.getCurrentPosition() * ticksToInch;

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

            double localx = (deltapod1 + deltapod2) / 2;
            double localy = deltapod3 - deltaheading * OdometryHorizontalOffset;

            if (deltaheading < OdometryHeadingThreshold) {
                x += localx * Math.cos(theta) - localy * Math.sin(theta);
                y += localy * Math.cos(theta) + localx * Math.sin(theta);

            } else {
                x += (localx * Math.sin(theta + deltaheading) + localy * Math.cos(theta + deltaheading)
                        - localx * Math.sin(theta) - localy * Math.cos(theta)) / deltaheading;
                y += (localy * Math.sin(theta + deltaheading) - localx * Math.cos(theta + deltaheading)
                        - localy * Math.sin(theta) + localx * Math.cos(theta)) / deltaheading;
            }

            theta += deltaheading;
            theta = theta % (PI * 2);
            if (theta < 0) theta += PI * 2;

            lastpod1 = pod1;
            lastpod2 = pod2;
            lastpod3 = pod3;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // odometry bulk read (old)
    /*public LynxGetBulkInputDataResponse revBulkData() {
        LynxGetBulkInputDataResponse response;
        try {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);
            response = command.sendReceive();
        } catch (Exception e) {
            opMode.telemetry.addData("Exception", "bulk read exception");
            response = null;
        }
        return response;
    }*/
}