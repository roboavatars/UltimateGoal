package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class StackHeightDetector {

    private OpenCvCamera cam;
    private StackHeightPipeline pipeline;

    public StackHeightDetector(LinearOpMode op) {

        int cameraMonitorViewId = op.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        pipeline = new StackHeightPipeline();

        cam.setPipeline(pipeline);
    }

    public void start() {
        cam.openCameraDeviceAsync(() ->
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
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

    public StackHeightPipeline.RingCase getResult() {
        return pipeline.getResult();
    }

    public double getRawResult() {
        return pipeline.getRawResult();
    }
}