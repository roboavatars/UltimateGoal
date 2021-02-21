package org.firstinspires.ftc.teamcode.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;

public class Dashboard {

    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static TelemetryPacket packet = new TelemetryPacket();

    public static void drawRobot(Robot robot, String color) {
        drawRobot(robot.x, robot.y, robot.theta, color);
    }

    public static void drawRobot(double robotX, double robotY, double robotTheta, String color) {
        double r = 9 * Math.sqrt(2);
        double x = robotY - 72;
        double y = 72 - robotX;
        double theta = PI/2 + robotTheta;
        double[] ycoords = {r * Math.sin(PI/4 + theta) + y, r * Math.sin(3*PI/4 + theta) + y, r * Math.sin(5*PI/4 + theta) + y, r * Math.sin(7*PI/4 + theta) + y};
        double[] xcoords = {r * Math.cos(PI/4 + theta) + x, r * Math.cos(3*PI/4 + theta) + x, r * Math.cos(5*PI/4 + theta) + x, r * Math.cos(7*PI/4 + theta) + x};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
    }

    public static void drawGoal(String color) {
        double[] xcoords = {72, 72, 78, 78};
        double[] ycoords = {-24, -48, -48, -24};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
    }

    public static void drawLine(double x1, double y1, double x2, double y2, String color) {
        packet.fieldOverlay().setStroke(color).strokeLine(y1 - 72, 72 - x1, y2 - 72, 72 - x2);
    }

    public static void drawRing(Ring ring) {
        drawRing(ring, "orange");
    }

    public static void drawRing(Ring ring, String color) {
        double x = ring.getY() - 72;
        double y = 72 - ring.getX();
        packet.fieldOverlay().setFill(color).fillCircle(x, y, 2.5);
        packet.fieldOverlay().setFill("white").fillCircle(x, y, 1.5);
    }

    public static void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public static void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
