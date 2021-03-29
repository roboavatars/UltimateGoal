package org.firstinspires.ftc.teamcode.RobotClasses;

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
    private T265Camera t265Cam;

    // Constants
    public static double ODOMETRY_COVARIANCE = 1;
    private final double INCH_TO_METER = 0.0254;
    private double xOffset = -5;
    private double yOffset = 3;

    // Position Variables
    private double x;
    private double y;
    private double theta;

    // Other
    public double confidence = 0;
//    private String mapPath = System.getProperty("java.io.tmpdir") + "/map.bin";
    private String mapPath = "/data/user/0/com.qualcomm.ftcrobotcontroller/cache/map.bin";
    public boolean isEmpty = false;
    private boolean exportingMap = false;

    public T265(LinearOpMode op, double startX, double startY, double startTheta) {

        File file = new File(mapPath);
        if (!file.exists() || file.length() == 0) {
            isEmpty = true;
        }

        Robot.log("isEmpty: " + isEmpty);

        if (!isEmpty) {
            t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, mapPath, op.hardwareMap.appContext);
        } else {
            t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, op.hardwareMap.appContext);
        }
        setCameraPose(startX, startY, startTheta);
    }

    public void startCam() {
        t265Cam.start((state) -> {
            Translation2d translation = new Translation2d(state.pose.getTranslation().getX() / INCH_TO_METER, state.pose.getTranslation().getY() / INCH_TO_METER);
            Rotation2d rotation = state.pose.getRotation();

            x = translation.getX();
            y = translation.getY();
            theta = rotation.getRadians();
        });
    }

    public void exportMap() {
        if (!exportingMap) {
            exportingMap = true;
            t265Cam.exportRelocalizationMap(mapPath);
            exportingMap = false;
        }
    }

    public void stopCam() {
//        exportMap();
        t265Cam.stop();
    }

    public void setCameraPose(double x, double y, double theta) {
//        double xPrime = yOffset * Math.cos(theta) + xOffset * Math.sin(theta);
//        double yPrime = yOffset * Math.sin(theta) - xOffset * Math.cos(theta);
//
//        t265Cam.setPose(new Pose2d((y - yPrime) * INCH_TO_METER, (xPrime - x) * INCH_TO_METER, new Rotation2d(theta - PI/2)));
        t265Cam.setPose(new Pose2d(x * INCH_TO_METER, y * INCH_TO_METER, new Rotation2d(theta)));

        Robot.log("inside t265 reset: " + x + ", " + y + ", " + theta);
    }

    public void sendOdometryData(double vx, double vy) {
        t265Cam.sendOdometry(vx, vy);
    }

    public void updateCamPose() {
        T265Camera.CameraUpdate temp = t265Cam.getLastReceivedCameraUpdate();
        if (temp.confidence == T265Camera.PoseConfidence.Low) {
            confidence = 1;
        } else if (temp.confidence == T265Camera.PoseConfidence.Medium) {
            confidence = 2;
        } else if (temp.confidence == T265Camera.PoseConfidence.High) {
            confidence = 3;
        } else {
            confidence = 0;
        }
    }

    public double getCamX() {
//        double yPrime = yOffset * Math.sin(theta) - xOffset * Math.cos(theta);
//        return -y - yPrime;
        return x;
    }

    public double getCamY() {
//        double xPrime = yOffset * Math.cos(theta) + xOffset * Math.sin(theta);
//        return x + xPrime;
        return y;
    }

    public double getCamTheta() {
        return theta;
    }

    public String confidenceColor() {
        if (confidence == 1) {
            return "orange";
        } else if (confidence == 2) {
            return "yellow";
        } else if (confidence == 3) {
            return "green";
        } else {
            return "red";
        }
    }
}