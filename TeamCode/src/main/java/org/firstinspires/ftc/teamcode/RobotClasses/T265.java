package org.firstinspires.ftc.teamcode.RobotClasses;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import java.io.File;

@SuppressWarnings("FieldCanBeLocal")
public class T265 {

    // Electronics
    private T265Camera t265Cam;

    // Constants
    public final double ODOMETRY_COVARIANCE = 0.1;
    private final double INCH_TO_METER = 0.0254;
    private final double xOffset = -9;
    private final double yOffset = 2;

    // Position Variables
    private double x;
    private double y;
    private double theta;

    // Other
    public double confidence = 0;
//    private final String mapPath = System.getProperty("java.io.tmpdir") + "/map.bin";
    private final String mapPath = "/data/user/0/com.qualcomm.ftcrobotcontroller/cache/map.bin";
    public boolean isEmpty = false;
    private boolean exportingMap = true;

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
        t265Cam.start();
    }

    public void exportMap() {
        if (exportingMap) {
            t265Cam.exportRelocalizationMap(mapPath);
            exportingMap = false;
        }
    }

    public void stopCam() {
//        exportMap();
        t265Cam.stop();
    }

    public void setCameraPose(double x, double y, double theta) {
        y -= xOffset * Math.cos(theta) - yOffset * Math.sin(theta);
        x -= -xOffset * Math.sin(theta) - yOffset * Math.cos(theta);

        t265Cam.setPose(new Pose2d(y * INCH_TO_METER, -x * INCH_TO_METER, new Rotation2d(theta)));
    }

    public void sendOdometryData(double vx, double vy) {
        t265Cam.sendOdometry(vy, -vx);
    }

    public void updateCamPose() {
        T265Camera.CameraUpdate state = t265Cam.getLastReceivedCameraUpdate();

        Translation2d translation = new Translation2d(state.pose.getTranslation().getX() / INCH_TO_METER, state.pose.getTranslation().getY() / INCH_TO_METER);
        Rotation2d rotation = state.pose.getRotation();

        y = translation.getX();
        x = -translation.getY();
        theta = rotation.getRadians();

        y += xOffset * Math.cos(theta) - yOffset * Math.sin(theta);
        x += -xOffset * Math.sin(theta) - yOffset * Math.cos(theta);

        if (state.confidence == T265Camera.PoseConfidence.Low) {
            confidence = 1;
        } else if (state.confidence == T265Camera.PoseConfidence.Medium) {
            confidence = 2;
        } else if (state.confidence == T265Camera.PoseConfidence.High) {
            confidence = 3;
        } else {
            confidence = 0;
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
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