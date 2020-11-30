package org.firstinspires.ftc.teamcode.RobotClasses;

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

    // Motors
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    // IMU and Rev Hub
    private LynxEmbeddedIMU imu;
    private LynxModule module;

    // OpMode
    private LinearOpMode opMode;

    // Tracking X Y coordinate position
    public double x = 0;
    public double y = 0;
    private double lastx = 0;
    private double lasty = 0;

    // IMU related variables for storing states
    private Orientation angles;
    private double lastheading = 0;
    private double deltaheading = 0;
    public double theta = 0;

    // Odometry
    public double pod1 = 0;
    public double pod2 = 0;
    public double pod3 = 0;
    public double deltapod1;
    public double deltapod2;
    public double deltapod3;
    private double lastpod1 = 0;
    private double lastpod2 = 0;
    private double lastpod3 = 0;

    // Motor Caching Stuff
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;
    private final double motorUpdateTolerance = 0.05;

    // Constants
    public static double OdometryTrackWidth = 13.79;
    public static double OdometryHorizontalOffset = -3.165;
    private final double OdometryHeadingThreshold = Math.PI / 8;

    public boolean zeroStrafeCorrection = false;

    // Constructor
    public MecanumDrivetrain(LinearOpMode opMode, double initialX, double initialY, double initialTheta) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        module = hardwareMap.get(LynxModule.class, "Control Hub");

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

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new LynxEmbeddedIMU(new BetterI2cDeviceSyncImplOnSimple(
                new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
        ));
        imu.initialize(new BNO055IMU.Parameters());

        x = initialX;
        y = initialY;
        lastheading = initialTheta;
        theta = initialTheta;
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

    // odometry bulk read
    public LynxGetBulkInputDataResponse revBulkData() {
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

    // update position from odometry
    public void updatePose() {
        try {
            LynxGetBulkInputDataResponse response = revBulkData();
            pod1 = -response.getEncoder(3) * 0.00300622055 * 2;
            pod2 = response.getEncoder(0) * 0.00300622055 * 2;
            pod3 = response.getEncoder(2) * 0.00300622055 * 2;

            deltapod1 = pod1 - lastpod1;
            deltapod2 = pod2 - lastpod2;
            deltapod3 = pod3 - lastpod3;

            if (deltapod1 == 0) Robot.log("pod 1 delta 0");
            if (deltapod2 == 0) Robot.log("pod 2 delta 0");
            if (deltapod3 == 0) Robot.log("pod 3 delta 0");

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

    // imu stuff (old)
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
    private static class BetterI2cDeviceSyncImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSyncImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }
        @Override public void setReadWindow(ReadWindow window) { /*intentionally do nothing*/}
    }
}