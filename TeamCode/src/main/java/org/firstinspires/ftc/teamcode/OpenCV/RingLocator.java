package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RingLocator {

    private OpenCvCamera cam;
    private RingLocatorPipeline pipeline;

    public RingLocator(LinearOpMode op) {

        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new RingLocatorPipeline();

        cam.setPipeline(pipeline);
    }

    public void start() {
        cam.openCameraDeviceAsync(() -> cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
        FtcDashboard.getInstance().startCameraStream(cam, 0);
    }

    public void stop() {
        cam.stopStreaming();
        cam.closeCameraDevice();
        FtcDashboard.getInstance().stopCameraStream();
    }

    public int getFrameCount() {
        return cam.getFrameCount();
    }

    public double getFPS() {
        return cam.getFps();
    }

    public int getTotalFrameTimeMS() {
        return cam.getTotalFrameTimeMs();
    }

    public int getPipelineTimeMS() {
        return cam.getPipelineTimeMs();
    }

    public int getOverheadTimeMS() {
        return cam.getOverheadTimeMs();
    }

    public int getCurrentPipelineMaxFPS() {
        return cam.getCurrentPipelineMaxFps();
    }

    public double[] getAbsRingPos(double robotX, double robotY, double robotTheta) {
        return pipeline.getAbsRingPos(robotX, robotY, robotTheta);
    }

    public double[] getRelRingPos() {
        return pipeline.getRelRingPos();
    }
}