package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class BaseDetector {

    private OpenCvCamera cam;

    public BaseDetector(LinearOpMode op) {
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
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

    public void setPipeline(OpenCvPipeline pipeline) {
        cam.setPipeline(pipeline);
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
}