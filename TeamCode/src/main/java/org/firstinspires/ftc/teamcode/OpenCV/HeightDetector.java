package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class HeightDetector {

    // Electronics
    private OpenCvCamera cam;

    private HeightDetectorPipeline pipeline;

    // OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public HeightDetector(LinearOpMode op) {
        this.op = op;
        this.hardwareMap = op.hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        pipeline = new HeightDetectorPipeline();

        cam.setPipeline(pipeline);
    }

    public void start() {
        cam.openCameraDeviceAsync(() ->
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
    }

    public void pause() {
        cam.stopStreaming();
    }

    public void stop() {
        pause();
        cam.closeCameraDevice();
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

    public double getResult() {
        return pipeline.getResult();
    }
}