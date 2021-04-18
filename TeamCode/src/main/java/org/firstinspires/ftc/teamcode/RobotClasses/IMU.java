package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.PI;

public class IMU {
    private BNO055IMU imu;
    private double angle;
    private double theta;
    private double lastHeading;
    private double deltaHeading = 0;

    public IMU(double startTheta, LinearOpMode op) {
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
            op.idle();
        }

        theta = startTheta;
        lastHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public void updateHeading() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        deltaHeading = angle - lastHeading;

        if (deltaHeading < -PI) {
            deltaHeading += 2*PI;
        } else if (deltaHeading >= PI) {
            deltaHeading -= 2*PI;
        }

        theta = (theta + deltaHeading) % (2*PI);
        if (theta < 0) {
            theta += 2*PI;
        }
        lastHeading = angle;
    }

    public void resetHeading(double newTheta) {
        lastHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        theta = newTheta;
    }

    public double getTheta() {
        return theta;
    }

    public double getDeltaHeading() {
        return deltaHeading;
    }
}
