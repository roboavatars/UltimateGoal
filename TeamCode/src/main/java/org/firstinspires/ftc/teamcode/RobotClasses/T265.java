package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

@SuppressWarnings("FieldCanBeLocal")
public class T265 {

    // Electronics
    private T265Camera t265Cam;

    // Constants
    public static double ODOMETRY_COVARIANCE = 1;
    private final double INCH_TO_METER = 0.0254;

    // Position Variables
    private double x;
    private double y;
    private double theta;

    // OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public T265(LinearOpMode op, double initX, double initY, double initTheta) {
        this.op = op;
        this.hardwareMap = op.hardwareMap;

        double xPrime = -0.3*Math.cos(initTheta) + 8.65*Math.sin(initTheta);
        double yPrime = -0.3*Math.sin(initTheta) - 8.65*Math.cos(initTheta);

        t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, hardwareMap.appContext);
        setCameraPose(initX, initY, initTheta);
    }

    public void startCam() {
        t265Cam.start((up) -> {
            Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / INCH_TO_METER, up.pose.getTranslation().getY() / INCH_TO_METER);
            Rotation2d rotation = up.pose.getRotation();

            x = translation.getX();
            y = translation.getY();
            theta = rotation.getRadians();
        });
    }

    public void stopCam() {
        t265Cam.stop();
    }

    public void setCameraPose(double x, double y, double theta) {
        double xPrime = -0.3 * Math.cos(theta) + 8.65 * Math.sin(theta);
        double yPrime = -0.3 * Math.sin(theta) - 8.65 * Math.cos(theta);

        t265Cam.setPose(new Pose2d((x + xPrime) * INCH_TO_METER, (y + yPrime) * INCH_TO_METER, new Rotation2d(theta - Math.PI/2)));
    }

    public void sendOdometryData (double vx, double vy) {
        t265Cam.sendOdometry(vx, vy);
    }

    public void updateCamPose() {
        t265Cam.getLastReceivedCameraUpdate();
    }

    public double getCamX() {
        double xPrime = -0.3*Math.cos(theta+Math.PI/2) + 8.65*Math.sin(theta+Math.PI/2);
        return x-xPrime;
    }

    public double getCamY() {
        double yPrime = -0.3*Math.sin(theta+Math.PI/2) - 8.65*Math.cos(theta+Math.PI/2);
        return y-yPrime;
    }

    public double getCamTheta() {
        return theta+Math.PI/2;
    }
}