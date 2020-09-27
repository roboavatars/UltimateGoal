package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    public MecanumDrivetrain drivetrain;
    public Shooter yeet;
    public IntakeIndex intakeIndex;

    private LinearOpMode op;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    public Robot(double x, double y, double theta, LinearOpMode op) {
        drivetrain = new MecanumDrivetrain(op, x,y, theta, true);
        yeet = new Shooter(op);
        intakeIndex = new IntakeIndex(op);

        this.op = op;
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        op.telemetry.update();
    }

    public void update() {
        drivetrain.updatePose();
        drawRobot(drivetrain.x, drivetrain.y, drivetrain.theta);
        addPacket("X", drivetrain.x);
        addPacket("Y", drivetrain.y);
        addPacket("Theta", drivetrain.theta);
        sendPacket();
    }

    public void shoot() { // aim for x=108,y=144

    }

    public void drawRobot(double robotX, double robotY, double robotTheta) {
        double r = 9 * Math.sqrt(2);
        double pi = Math.PI;
        double x = 72 - robotY;
        double y = robotX - 72;
        double theta = pi/2 + robotTheta;
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
