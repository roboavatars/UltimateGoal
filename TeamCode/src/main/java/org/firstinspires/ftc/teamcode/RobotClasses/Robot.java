package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("FieldCanBeLocal")
public class Robot {

    // Robot Classes
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Shooter shooter;
    public T265 t265;
    public Logger logger;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final int distUpdatePeriod = 15;

    // State Variables
    private boolean firstLoop = true;
    private int cycleCounter = 0;
    private int numRings = 0;

    private double shootTime;
    private boolean timeSaved = false;

    public boolean shoot = false;

    // Motion Variables
    private double x, y, theta, prevX, prevY, prevTheta, vx, vy, w, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // Shooter Variables
    private final double[] shootX = {76.5, 84, 91.5, 108};
    private final double shootY = 144;
    private final double[] shootZ = {24, 24, 24, 35.5};

    private final double feedShootDelay = 100;
    private final double feedHomeDelay = 100;

    // OpMode Stuff
    private LinearOpMode op;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta) {
        drivetrain = new MecanumDrivetrain(op, x, y, theta, true);
        intake = new Intake(op);
        shooter = new Shooter(op);
        t265 = new T265(op, x, y, theta);
        logger = new Logger();

        this.op = op;
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    public void update() {
        cycleCounter++;

        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        if (cycleCounter % distUpdatePeriod == 0) {
            numRings = shooter.ringsInMag();
        }

        /*
        States (in progress)-
        <3 rings, no shoot, mag home, feed home- nothing (default state)
        3 rings, no shoot, mag home, feed home- mag to shoot, intake off

        >0 rings, shoot, mag shoot, feed home- feed to shoot, save time (maybe also dt align, servo angle, flywheel)
        >0 rings, shoot, mag shoot, feed shoot, delay passed- feed to home, rings-1
        need delay before shoot again

        0 rings, shoot, mag shoot, feed home- mag home, intake on
        */

        if (numRings == 3 && !shoot && shooter.magHome && shooter.feedHome) {
            shooter.magShoot();
            intake.intakeOff();
        }

        else if (numRings > 0 && shoot && !shooter.magHome && shooter.feedHome) {
            if (!timeSaved || (timeSaved && (System.currentTimeMillis()-shootTime)>feedHomeDelay)) {
                shooter.feedShoot();
                shootTime = System.currentTimeMillis();
                timeSaved = true;
            }
        }

        else if (numRings > 0 && shoot && !shooter.magHome && !shooter.feedHome && (System.currentTimeMillis()-shootTime)>feedShootDelay) {
            shooter.feedHome();
            numRings--;
        }

        else if (numRings == 0 && shoot && !shooter.magHome && shooter.feedHome) {
            shooter.magHome();
            shoot = false;
            timeSaved = false;
            intake.intakeOn();
        }

        // Update Position
        drivetrain.updatePose();
        t265.updateCamPose();

        // Calculate Motion Info
        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
//        x = (drivetrain.x + t265.getCamX()) / 2;
//        y = (drivetrain.y + t265.getCamY()) / 2;
//        theta = (drivetrain.theta + t265.getCamTheta()) / 2;
        x = t265.getCamX();
        y = t265.getCamY();
        theta = t265.getCamTheta();
        vx = (x - prevX) / timeDiff;
        vy = (y - prevY) / timeDiff;
        w = (theta - prevTheta) / timeDiff;
        ax = (vx - prevVx) / timeDiff;
        ay = (vy - prevVy) / timeDiff;
        a = (w - prevW) / timeDiff;

        // Log Data
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime, x, y, theta, vx, vy, w, ax, ay, a);
        }

        // Remember Old Motion Info
        prevX = x; prevY = y; prevTheta = theta;
        prevTime = curTime;
        prevVx = vx; prevVy = vy; prevW = w;

        // Telemetry
        addPacket("X", x);
        addPacket("Y", y);
        addPacket("Theta", theta);
        addPacket("Angle Pos", shooter.angleServo.getPosition());
        addPacket("Update Frequency (Hz)", 1 / timeDiff);
        drawGoal(x, y, theta, "black");
        drawRobot(x, y, theta, "black");
        double[] calculated = shoot(3);
        drawRobot(x, y, calculated[0], "green");
        sendPacket();
    }

    // left ps = 0, middle ps = 1, right ps = 2, high goal = 3
    public double[] shoot(int targetNum) {
        /*  power1- (76.5,144,24)
            power2- (84,144,24)
            power3- (91.5,144,24)
            high goal- (108,144,35.5)
        */

        double targetX = shootX[targetNum];
        double targetY = shootY;
        double targetZ = shootZ[targetNum];
        double robotX = x;
        double robotY = y;

        // Calculate Robot Angle
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double v = 63; // tangential velocity of ring when leaving flywheel
        double p = v * dy;
        double q = -v * dx;
        double alignRobotAngle = Math.asin(((targetX - robotX) * vx - (targetY - robotY) * vy) / Math.sqrt(Math.pow(p, 2) + Math.pow(q, 2))) - Math.atan(q / p);
        if (robotX >= targetX) {
            alignRobotAngle += Math.PI;
        }
        addPacket("Robot Angle", alignRobotAngle);

        // Calculate Shooter Angle
        double d = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
        double dz = targetZ - 8;
        double a = (386 * Math.pow(d, 2)) / (2 * Math.pow(v, 2));
        double b = d;
        double c = -dz - a;
        double quadraticRes = (-b - Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
        double shooterAngle = 0.27 * (Math.atan(quadraticRes) - 5 * Math.PI / 36) * 3 / Math.PI;
        addPacket("Shooter Angle", shooterAngle);

        return new double[] {alignRobotAngle, shooterAngle};
    }

    public void drawGoal(double x, double y, double theta, String color) {
        double[] xcoords = {72, 72, 78, 78};
        double[] ycoords = {-24, -48, -48, -24};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
        packet.fieldOverlay().setStroke(color).strokeLine(72 - x, 72 - y, 72, -36);
    }

    public void drawRobot(double robotX, double robotY, double robotTheta, String color) {
        double r = 9 * Math.sqrt(2);
        double pi = Math.PI;
        double x = 72 - robotX;
        double y = 72 - robotY;
        double theta = pi/2 + robotTheta;
        double[] ycoords = {r * Math.sin(pi/4 + theta) + y, r * Math.sin(3 * pi/4 + theta) + y, r * Math.sin(5 * pi/4 + theta) + y, r * Math.sin(7 * pi/4 + theta) + y};
        double[] xcoords = {r * Math.cos(pi/4 + theta) + x, r * Math.cos(3 * pi/4 + theta) + x, r * Math.cos(5 * pi/4 + theta) + x, r * Math.cos(7 * pi/4 + theta) + x};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
    }

    public void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
