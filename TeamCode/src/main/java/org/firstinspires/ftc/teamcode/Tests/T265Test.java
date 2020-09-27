package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.RobotClasses.T265;

@TeleOp(name="T265 Test")
public class T265Test extends LinearOpMode {

    private T265 t265;
    private double x, y, theta;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    @Override
    public void runOpMode() {
        t265 = new T265(this, 9, 111, Math.PI/2);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        waitForStart();
        t265.startCam();

        while(opModeIsActive()){
            t265.updateCamPose();
            x = t265.getCamX();
            y = t265.getCamY();
            theta = t265.getCamTheta();

            drawRobot(x, y, theta);
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            sendPacket();

            telemetry.addData("X: " , x);
            telemetry.addData("Y: " , y);
            telemetry.addData("Theta: " , theta);
            telemetry.update();
        }

        t265.stopCam();
    }

    public void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public void drawRobot(double robotx, double roboty, double robottheta) {
        double r = 9 * Math.sqrt(2);
        double pi = Math.PI;
        double x = 72 - roboty;
        double y = robotx - 72;
        double theta = pi/2 + robottheta;
        double[] ycoords = {r * Math.sin(pi/4 + theta) + y, r * Math.sin(3 * pi/4 + theta) + y, r * Math.sin(5 * pi/4 + theta) + y, r * Math.sin(7 * pi/4 + theta) + y};
        double[] xcoords = {r * Math.cos(pi/4 + theta) + x, r * Math.cos(3 * pi/4 + theta) + x, r * Math.cos(5 * pi/4 + theta) + x, r * Math.cos(7 * pi/4 + theta) + x};
        packet.fieldOverlay().setFill("green").fillPolygon(xcoords,ycoords);
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}