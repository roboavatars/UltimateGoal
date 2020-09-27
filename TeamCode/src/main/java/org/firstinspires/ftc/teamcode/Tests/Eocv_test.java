package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@TeleOp
public class Eocv_test extends LinearOpMode {

    OpenCvCamera phoneCam;
    StackPipe pipe;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        pipe = new StackPipe();
        phoneCam.setPipeline(pipe);

        phoneCam.openCameraDeviceAsync(() ->
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        addPacket("status", "ready"); sendPacket();
        waitForStart();

        while (opModeIsActive()) {

            addPacket("Frame Count", phoneCam.getFrameCount());
            addPacket("FPS", phoneCam.getFps());
            addPacket("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            addPacket("Pipeline time ms", phoneCam.getPipelineTimeMs());
            addPacket("Overhead time ms", phoneCam.getOverheadTimeMs());
            addPacket("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            addPacket("z Result", pipe.getResult());

            sendPacket();
            sleep(100);
        }

    }

    public void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
