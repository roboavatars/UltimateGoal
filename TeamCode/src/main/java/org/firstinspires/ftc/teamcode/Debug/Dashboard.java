package org.firstinspires.ftc.teamcode.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static org.firstinspires.ftc.teamcode.RobotClasses.Shooter.TURRET_DX;
import static org.firstinspires.ftc.teamcode.RobotClasses.Shooter.TURRET_DY;
import static org.firstinspires.ftc.teamcode.RobotClasses.Shooter.TURRET_DIAMETER;

public class Dashboard {

    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static TelemetryPacket packet = new TelemetryPacket();

    public static void drawRobot(Robot robot) {
        drawRobot(robot, "black", "gray");
    }

    public static void drawRobot(Robot robot, String drivetrainColor, String turretColor) {
        drawRobot(robot.x, robot.y, robot.theta, robot.turretGlobalTheta, drivetrainColor, turretColor);
    }

    public static void drawRobot(double robotX, double robotY, double robotTheta, double turretTheta, String drivetrainColor, String turretColor) {
        drawDrivetrain(robotX, robotY, robotTheta, drivetrainColor);
        drawTurret(robotX, robotY, robotTheta, turretTheta, turretColor);
    }

    public static void drawDrivetrain(double robotX, double robotY, double robotTheta, String robotColor) {
        double r = 9 * Math.sqrt(2);
        double x = robotY - 72;
        double y = 72 - robotX;
        double theta = PI/2 + robotTheta;
        double[] xcoords = {r * cos(PI/4 + theta) + x, r * cos(3*PI/4 + theta) + x, r * cos(5*PI/4 + theta) + x, r * cos(7*PI/4 + theta) + x};
        double[] ycoords = {r * sin(PI/4 + theta) + y, r * sin(3*PI/4 + theta) + y, r * sin(5*PI/4 + theta) + y, r * sin(7*PI/4 + theta) + y};
        packet.fieldOverlay().setFill(robotColor).fillPolygon(xcoords, ycoords);
    }

    public static void drawTurret(double robotX, double robotY, double robotTheta, double turretTheta, String turretColor) {
        double cx = robotY - TURRET_DX * cos(robotTheta) + TURRET_DY * sin(robotTheta);
        double cy = robotX + TURRET_DX * sin(robotTheta) + TURRET_DY * cos(robotTheta);
        double r = TURRET_DIAMETER / 2;
        packet.fieldOverlay().setFill(turretColor).fillCircle(cy - 72, 72 - cx, r);
        drawRect(robotX - r * cos(turretTheta - PI/2) / 2, robotY - r * sin(turretTheta - PI/2) / 2, robotX + r * cos(turretTheta - PI/2) / 2 + cos(turretTheta), robotY + r * sin(turretTheta - PI/2) / 2 + sin(turretTheta), turretColor);
    }

    public static void drawField() {
        // Perimeter
        outlineRect(0, 0, 144, 144, "black");

        // Tower Goals
        drawRect(24, 144, 48, 150, "black");
        drawRect(96, 144, 120, 150, "black");

        // Blue Powershots
        drawRect(67, 144, 68, 147.5, "blue");
        drawRect(59.5, 144, 60.5, 147.5, "blue");
        drawRect(52, 144, 53, 147.5, "blue");

        // Red Powershots
        drawRect(76, 144, 77, 147.5, "red");
        drawRect(83.5, 144, 84.5, 147.5, "red");
        drawRect(91, 144, 92, 147.5, "red");
    }

    public static void drawPoint(double x, double y, String color) {
        packet.fieldOverlay().setFill(color).fillCircle(y - 72, 72 - x, 0.5);
    }

    public static void drawLine(double x1, double y1, double x2, double y2, String color) {
        packet.fieldOverlay().setStroke(color).strokeLine(y1 - 72, 72 - x1, y2 - 72, 72 - x2);
    }

    public static void outlineRect(double x1, double y1, double x2, double y2, String color) {
        double[] xcoords = {y1 - 72, y2 - 72, y2 - 72, y1 - 72};
        double[] ycoords = {72 - x1, 72 - x1, 72 - x2, 72 - x2};
        packet.fieldOverlay().setStroke(color).strokePolygon(xcoords, ycoords);
    }

    public static void drawRect(double x1, double y1, double x2, double y2, String color) {
        double[] xcoords = {y1 - 72, y2 - 72, y2 - 72, y1 - 72};
        double[] ycoords = {72 - x1, 72 - x1, 72 - x2, 72 - x2};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
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
