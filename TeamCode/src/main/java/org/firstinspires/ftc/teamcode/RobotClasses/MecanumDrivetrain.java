package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("FieldCanBeLocal")
public class MecanumDrivetrain {

    //1440 ticks per encoder revolution
    //4.3289 inches per wheel revolution
    double encoderCountsPerRevolution = 537.6;

    // Motors of the drivetrain
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    // OpMode Stuff
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    // Objects for IMU and Rev Hub
    private LynxEmbeddedIMU imu;
    private LynxModule module;

    // IMU related variables for storing states
    private Orientation angles;
    private double lastheading = 0;
    private double deltaheading = 0;
    public double theta = 0;

    // Tracking X Y coordinate position
    public double x = 0;
    public double y = 0;
    private double lastpod1 = 0;
    private double lastpod2 = 0;
    private double lastpod3 = 0;

    // K Variables for Control of Linear System
    private double xk = 0.15;
    private double yk = 0.15;
    private double thetak = 0.95;

    // Constants
    private final double xyTolerance = 1;
    private final double thetaTolerance = Math.PI / 35;
    private final double OdometryTrackWidth = 13.85;
    private double OdometryHorizontalOffset = 3.165;
    private final double OdometryHeadingThreshold = Math.PI / 8;

    public double deltapod1;
    public double deltapod2;
    public double deltapod3;
    private double lastx = 0;
    private double lasty = 0;

    private boolean isRed;

    // Motor Caching Stuff
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;
    public static double motorUpdateTolerance = 0.05;

    public double pod1 = 0;
    public double pod2 = 0;
    public double pod3 = 0;

    public boolean zeroStrafeCorrection = false;

    // Constructor
    public MecanumDrivetrain(LinearOpMode opMode, double initialX, double initialY, double initialTheta, boolean isRedAuto) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        module = hardwareMap.get(LynxModule.class, "Drivetrain Hub");

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

        imu = new LynxEmbeddedIMU(new MecanumDrivetrain.BetterI2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
        ));

        imu.initialize(new BNO055IMU.Parameters());

        x = initialX;
        y = initialY;
        lastheading = initialTheta;
        theta = initialTheta;

        isRed = isRedAuto;

        if (!isRed) {
            thetak = 1.15;
        }

        opMode.telemetry.addLine("ExH Version: " + getConciseLynxFirmwareVersion(module));
        opMode.telemetry.update();
    }

    private class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }

    private static String getConciseLynxFirmwareVersion(LynxModule module) {
        String rawVersion = module.getFirmwareVersionString();
        String[] parts = rawVersion.split(" ");
        StringBuilder versionBuilder = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            String part = parts[3 + 2 * i];
            if (i == 2) {
                versionBuilder.append(part);
            } else {
                versionBuilder.append(part, 0, part.length() - 1);
                versionBuilder.append(".");
            }
        }
        return versionBuilder.toString();
    }

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

    public void setTargetPoint(double xtarget, double ytarget, double thetatarget) {
        // Make Sure thetatarget is Between 0 and 2pi
        thetatarget = thetatarget % (Math.PI * 2);
        if (thetatarget < 0) {
            thetatarget += Math.PI * 2;
        }

        // Picking the Smaller Distance to Rotate
        double thetacontrol = 0;
        if (theta - thetatarget > Math.PI) {
            thetacontrol = theta - thetatarget - 2 * Math.PI;
        } else if (theta - thetatarget < (-Math.PI)) {
            thetacontrol = theta - thetatarget + 2 * Math.PI;
        } else {
            thetacontrol = theta - thetatarget;
        }
        //Log.w("auto", "thetacontrol: " + thetacontrol);

        setGlobalControls(-xk * (x - xtarget), -yk * (y - ytarget), -thetak * (thetacontrol));
    }

    public void setTargetPoint(double xtarget, double ytarget, double thetatarget, double xK, double yK, double thetaK) {
        // Make Sure thetatarget is Between 0 and 2pi
        thetatarget = thetatarget % (Math.PI * 2);
        if (thetatarget < 0) {
            thetatarget += Math.PI * 2;
        }

        // Picking the Smaller Distance to Rotate
        double thetacontrol = 0;
        if (Math.abs(theta - thetatarget) > Math.PI) {
            thetacontrol = theta - thetatarget - 2 * Math.PI;
        } else {
            thetacontrol = theta - thetatarget;
        }

        setGlobalControls(-xK * (x - xtarget), -yK * (y - ytarget), -thetaK * (thetacontrol));
    }

    public void setTargetPointAuto(double xtarget, double ytarget, double thetatarget) {
        if (!isRed) {
            xtarget = 142 - xtarget;
            thetatarget = (Math.PI) - thetatarget;
        }

        setTargetPoint(xtarget, ytarget, thetatarget);
    }

    public void setTargetPointAuto(double xtarget, double ytarget, double thetatarget, double xK, double yK, double thetaK) {
        if (!isRed) {
            xtarget = 142 - xtarget;
            thetatarget = (Math.PI) - thetatarget;
        }

        setTargetPoint(xtarget, ytarget, thetatarget, xK, yK, thetaK);
    }

    public void setGlobalControls(double xvelocity, double yvelocity, double w) {
        double xdot = xvelocity * Math.cos(-theta) - yvelocity * Math.sin(-theta);
        double ydot = yvelocity * Math.cos(-theta) + xvelocity * Math.sin(-theta);
        setControls(xdot, ydot, w);
    }

    public LynxGetBulkInputDataResponse RevBulkData() {
        LynxGetBulkInputDataResponse response;
        try {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);
            response = command.sendReceive();
        } catch (Exception e) {
            opMode.telemetry.addData("Exception", "bulk read exception");
            response = null;
        }
        return response;
    }

    public void updatePose() {
        try {
            LynxGetBulkInputDataResponse response = RevBulkData();
            pod1 = -response.getEncoder(3) * 0.00300622055 * 2;
            pod2 = response.getEncoder(0) * 0.00300622055 * 2;
            pod3 = -response.getEncoder(2) * 0.00300622055 * 2;

            deltapod1 = pod1 - lastpod1;
            deltapod2 = pod2 - lastpod2;
            deltapod3 = pod3 - lastpod3;

            Log.w("auto", deltapod1 + " " + deltapod2 + " " + deltapod3);
            if (deltapod1 == 0) Log.w("auto", "pod 1 delta 0");
            if (deltapod2 == 0) Log.w("auto", "pod 2 delta 0");
            if (deltapod3 == 0) Log.w("auto", "pod 3 delta 0");

            lastx = x;
            lasty = y;

            deltaheading = (deltapod1 - deltapod2) / OdometryTrackWidth;

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
            theta = theta % (Math.PI * 2);
            if (theta < 0) theta += Math.PI * 2;

            lastpod1 = pod1;
            lastpod2 = pod2;
            lastpod3 = pod3;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public double getHeadingImu() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        deltaheading = angles.firstAngle - lastheading;

        //opMode.telemetry.addData("delta", deltaheading);

        if (deltaheading < -Math.PI)
            deltaheading += 2 * Math.PI;
        else if (deltaheading >= Math.PI)
            deltaheading -= 2 * Math.PI;

        theta += deltaheading;
        lastheading = angles.firstAngle;

        return theta;
    }

    public void resetHeadingIMU() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        lastheading = angles.firstAngle;
        theta = 0;
    }

    public boolean isAtPoseAuto(double targetx, double targety, double targettheta) {
        return isAtPoseAuto(targetx, targety, targettheta, xyTolerance, xyTolerance, thetaTolerance);
    }

    public boolean isAtPoseAuto(double targetx, double targety, double targettheta, double xtolerance, double ytolerance, double thetatolerance) {
        if (!isRed) {
            targetx = 144 - targetx;
            targettheta = Math.PI - targettheta;
        }
        return (Math.abs(x - targetx) < xtolerance && Math.abs(y - targety) < ytolerance && Math.abs(theta - targettheta) < thetatolerance);
    }
}