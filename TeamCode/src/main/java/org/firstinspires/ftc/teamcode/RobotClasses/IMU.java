package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.PI;

@Config
public class IMU {
    private BNO055IMU imu;
    private BNO055IMU imu2;
    private double angle;
    private double theta;
    private double lastHeading;
    private double lastHeading2;
    private double deltaHeading = 0;
    private double deltaHeading2 = 0;

    public static double IMU_DRIFT_COMPENSATION = 1.0018416;

    public IMU(double startTheta, LinearOpMode op) {
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        imu2 = op.hardwareMap.get(BNO055IMU.class, "imu2");
        imu2.initialize(new BNO055IMU.Parameters());

        while (!op.isStopRequested() && (!imu.isGyroCalibrated() || !imu2.isGyroCalibrated())) {
            op.idle();
        }

        theta = startTheta;
        double lastHeading1 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double lastHeading2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        lastHeading = (lastHeading1 + lastHeading2) / 2;
    }

    public void updateHeading() {
        double angle1 = IMU_DRIFT_COMPENSATION * imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double angle2 = IMU_DRIFT_COMPENSATION * imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        deltaHeading = angle1 - lastHeading;
        deltaHeading2 = angle2 - lastHeading2;

        if (deltaHeading < -PI) {
            deltaHeading += 2*PI;
        } else if (deltaHeading >= PI) {
            deltaHeading -= 2*PI;
        }
        if (deltaHeading2 < -PI) {
            deltaHeading2 += 2*PI;
        } else if (deltaHeading2 >= PI) {
            deltaHeading2 -= 2*PI;
        }

        theta = (theta + (deltaHeading + deltaHeading2) / 2) % (2*PI);
        if (theta < 0) {
            theta += 2*PI;
        }
        lastHeading = angle1;
        lastHeading2 = angle2;
    }

    public void updateHeadingUncapped() {
        double angle1 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double angle2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        deltaHeading = angle1 - lastHeading;
        deltaHeading2 = angle2 - lastHeading2;

        if (deltaHeading < -PI) {
            deltaHeading += 2*PI;
        } else if (deltaHeading >= PI) {
            deltaHeading -= 2*PI;
        }
        if (deltaHeading2 < -PI) {
            deltaHeading2 += 2*PI;
        } else if (deltaHeading2 >= PI) {
            deltaHeading2 -= 2*PI;
        }

        theta += (deltaHeading + deltaHeading2) / 2;
        lastHeading = angle1;
        lastHeading2 = angle2;
    }

    public void resetHeading(double newTheta) {
        double lastHeading1 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double lastHeading2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        lastHeading = (lastHeading1 + lastHeading2) / 2;
        theta = newTheta;
    }

    public double getTheta() {
        return theta;
    }

    public double getDeltaHeading() {
        return deltaHeading;
    }
}
