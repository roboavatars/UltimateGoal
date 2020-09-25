package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    public MecanumDrivetrain drivetrain;
    public Shooter yeet;
    public Intake intake;

    private LinearOpMode op;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    public Robot(double x, double y, double theta, LinearOpMode op) {
        drivetrain = new MecanumDrivetrain(op, x,y, theta, true);
        yeet = new Shooter();
        intake = new Intake();

        this.op = op;
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    public void update() {
        drivetrain.updatePose();
        drawRobot(drivetrain.x, drivetrain.y, drivetrain.theta);
        addPacket("X", drivetrain.x);
        addPacket("Y", drivetrain.y);
        addPacket("Theta", drivetrain.theta);
        sendPacket();
    }

    public void drawRobot(double robotx, double roboty, double robottheta) {
        double r = 9 * Math.sqrt(2);
        double pi = Math.PI;
        double x = 72 - roboty;
        double y = robotx - 72;
        double theta = pi/2 + robottheta;
        double[] ycoords = {r * Math.sin(pi/4 + theta) + y, r * Math.sin(3 * pi/4 + theta) + y, r * Math.sin(5 * pi/4 + theta) + y, r * Math.sin(7 * pi/4 + theta) + y};
        double[] xcoords = {r * Math.cos(pi/4 + theta) + x, r * Math.cos(3 * pi/4 + theta) + x, r * Math.cos(5 * pi/4 + theta) + x, r * Math.cos(7 * pi/4 + theta) + x};
        packet.fieldOverlay().fillPolygon(xcoords,ycoords);
    }

    public void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
