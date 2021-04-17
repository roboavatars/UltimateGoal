package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.PI;

public class IMU {

    private BNO055IMU imu;
    private Orientation angles;
    private double theta = PI/2;
    private double lastHeading = 0;
    private double deltaHeading = 0;

    public IMU(double startTheta, LinearOpMode op) {
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
            op.idle();
        }
    }

    public void updateHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        deltaHeading = angles.firstAngle - lastHeading;

        theta += deltaHeading;

        theta = theta % (2*PI);
        if (theta < 0) theta += 2*PI;

        lastHeading = angles.firstAngle;
    }

    public void resetHeading(double newTheta) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        lastHeading = angles.firstAngle;
        theta = newTheta;
    }

    public double getTheta() {
        return theta;
    }

    public double getDeltaHeading() {
        return deltaHeading;
    }
}
