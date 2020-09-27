package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.HeightDetector;

@TeleOp(name = "OpenCV Height Detector Test")
public class HeightDetectorTest extends LinearOpMode {

    private HeightDetector detector;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    @Override
    public void runOpMode() {
        detector = new HeightDetector(this);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        waitForStart();
        detector.start();

        while (opModeIsActive()) {
            addPacket("Frame Count", detector.getFrameCount());
            addPacket("FPS", detector.getFPS());
            addPacket("Result", detector.getResult());
            sendPacket();
        }

        detector.stop();
    }

    public void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}