package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import java.io.File;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal")
public class T265 {

    // Electronics
    private static T265Camera t265Cam;

    // Constants
    public final double ODOMETRY_COVARIANCE = 0.1;
    private final double INCH_TO_METER = 0.0254;
    private final double xOffset = -9;
    private final double yOffset = 2;

    // State Variables
    private double x, y, theta;
    private double startTheta;
    public double thetaError = 0;
    private double camInitTheta;
    public int confidence = 0;

    @SuppressLint("SdCardPath")
    private final String mapPath = "/data/user/0/com.qualcomm.ftcrobotcontroller/cache/map.bin";
    public boolean isEmpty = false;
    private boolean exportingMap = true;

    public T265(LinearOpMode op, double startX, double startY, double startTheta) {
        File file = new File(mapPath);
        if (!file.exists() || file.length() == 0) {
            isEmpty = true;
        }

        this.startTheta = startTheta;

        Robot.log(isEmpty ? "T265 Localization Map Not Found" : "Found T265 Localization Map");
        Robot.log(t265Cam == null ? "Instantiating new T265" : "Using static T265");

        if (t265Cam == null) {
            if (!isEmpty) {
                t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, mapPath, op.hardwareMap.appContext);
            } else {
                t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, op.hardwareMap.appContext);
            }
        }
        setCameraPose(startX, startY, startTheta);
    }

    public void startCam() {
        if (!t265Cam.isStarted()) {
            t265Cam.start();
            updateCamPose();
        } else {
            stopCam();
            startCam();
        }
    }

    public void exportMap() {
        if (exportingMap) {
            exportingMap = false;
            t265Cam.exportRelocalizationMap(mapPath);
        }
    }

    public void stopCam() {
        t265Cam.stop();
    }

    public void setCameraPose(double x, double y, double theta) {
        x -= -xOffset * Math.sin(theta) - yOffset * Math.cos(theta);
        y -= xOffset * Math.cos(theta) - yOffset * Math.sin(theta);

        t265Cam.setPose(new Pose2d(y * INCH_TO_METER, -x * INCH_TO_METER, new Rotation2d(theta)));
    }

    public void sendOdometryData(double vx, double vy, double theta, double w) {
        double r = Math.hypot(xOffset, yOffset);
        theta += Math.atan2(yOffset, xOffset) - PI/2;
        t265Cam.sendOdometry(vy + r * w * Math.sin(theta), -vx - r * w * Math.cos(theta));
    }

    public void updateCamPose() {
        T265Camera.CameraUpdate state = t265Cam.getLastReceivedCameraUpdate();

        Translation2d translation = new Translation2d(state.pose.getTranslation().getX() / INCH_TO_METER, state.pose.getTranslation().getY() / INCH_TO_METER);
        Rotation2d rotation = state.pose.getRotation();

        x = -translation.getY() - xOffset * Math.sin(theta) - yOffset * Math.cos(theta);
        y = translation.getX() + xOffset * Math.cos(theta) - yOffset * Math.sin(theta);
        theta = rotation.getRadians();

        if (state.confidence == T265Camera.PoseConfidence.High) {
            confidence = 3;
        } else if (state.confidence == T265Camera.PoseConfidence.Medium) {
            confidence = 2;
        } else if (state.confidence == T265Camera.PoseConfidence.Low) {
            confidence = 1;
        } else {
            confidence = 0;
        }
    }

    public double getThetaError() {
        return thetaError;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getRawTheta() {
        return theta;
    }

    public double getTheta() {
        return theta + thetaError;
    }

    public void makeSureT265IsGood() {
        updateCamPose();
        camInitTheta = theta;
        thetaError = startTheta - camInitTheta;
        Robot.log("Initial T265 pos: " + camInitTheta);
        Robot.log("Initial T265 error: " + thetaError);
    }

    public void resetTheta() {
        thetaError = startTheta - theta;
    }

    public double getInitTheta() {
        return camInitTheta;
    }

    public String confidenceColor() {
        switch (confidence) {
            case 3:
                return "green";
            case 2:
                return "yellow";
            case 1:
                return "orange";
            default :
                return "red";
        }
    }
}