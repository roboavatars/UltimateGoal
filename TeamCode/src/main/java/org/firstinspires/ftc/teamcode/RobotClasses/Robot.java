package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Arrays;

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
    //private final int distUpdatePeriod = 15;
    private final double xyTolerance = 1;
    private final double thetaTolerance = Math.PI / 35;
    private final double odoWeight = 1;
    private final double camWeight = 0;

    // State Variables
    private boolean firstLoop = true;
    private int cycleCounter = 0;
    public int numRings = 0;

    public double shootCounter = 0;
    public double shootDelay = 100;
    public boolean shoot = false;
    public boolean clear = false;
    public boolean highGoal = false;

    // Motion Variables
    public double x, y, theta;
    private double prevX, prevY, prevTheta, vx, vy, w, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    private double startTime;

    // Shooter Variables
    private final double[] shootX = {76.5, 84, 91.5, 108};
    private final double shootY = 150;
    private final double[] shootZ = {24, 24, 24, 35.5};

    double[][] targets = {{90.003147736901, 67.797736134046, 1.7463062382935, 0.0307346223878468},
            {90.011991547720, 68.394647086835, 1.6709188686716, 0.035584947098101},
            {90.076193524281, 68.992325734076, 1.4275314990497, 0.038232276827628}};

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
        shootCounter++;

        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        /*if (cycleCounter % distUpdatePeriod == 0) {
            numRings = shooter.ringsInMag();
        }*/

        /*
        States (in progress)-
        <3 rings, no shoot, mag home, feed home- nothing (default state)
        3 rings, no shoot, mag home, feed home- mag to shoot, intake off

        >0 rings, shoot, mag shoot, feed home- feed to shoot, save time (maybe also dt align, servo angle, flywheel)
        >0 rings, shoot, mag shoot, feed shoot, delay passed- feed to home, rings-1
        need delay before shoot again

        0 rings, shoot, mag shoot, feed home- mag home, intake on
        */

        if (numRings == 3 && shoot && shooter.magHome && shooter.feedHome) {
            shooter.magShoot();
            intake.intakeOff();
        }

        else if (numRings >= 0 && shoot && !shooter.magHome) {
            if (shootCounter % shootDelay == 0) {
                if (numRings > 0) {
                    if (shooter.feedHome && !clear) {
                        clear = true;
                        shooter.feedShoot();
                    } else {
                        shooter.feedHome();
                        clear = false;
                        numRings--;
                    }
                } else {
                    shooter.flywheelOff();
                    shooter.magHome();
                    shoot = false;
                }
            }

            if (numRings > 0) {
                double[] target;
                if (highGoal) {
                    target = shoot(3);
                } else {
                    target = targets[numRings - 1];
                }
                setTargetPoint(target[0], target[1], target[2]);
                shooter.setFlapAngle(target[3]);
            }
        }

        // Update Position
        drivetrain.updatePose();
//        t265.sendOdometryData(vx, vy);
        t265.updateCamPose();

        // Calculate Motion Info
        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        x = odoWeight * drivetrain.x + camWeight * t265.getCamX();
        y = odoWeight * drivetrain.y + camWeight * t265.getCamY();
        theta = odoWeight * drivetrain.theta + camWeight * t265.getCamTheta();
//        x = drivetrain.x;//*/ t265.getCamX();
//        y = drivetrain.y;//*/ t265.getCamY();
//        theta = drivetrain.theta;//*/ t265.getCamTheta();
        vx = (x - prevX) / timeDiff;
        vy = (y - prevY) / timeDiff;
        w = (theta - prevTheta) / timeDiff;
        ax = (vx - prevVx) / timeDiff;
        ay = (vy - prevVy) / timeDiff;
        a = (w - prevW) / timeDiff;

        // Log Data
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime, x, y, theta, vx, vy, w, ax, ay, a, numRings);
        }

        // Remember Old Motion Info
        prevX = x; prevY = y; prevTheta = theta;
        prevTime = curTime;
        prevVx = vx; prevVy = vy; prevW = w;

        // Telemetry
        addPacket("1 X", x);
        addPacket("2 Y", y);
        addPacket("3 Theta", theta);
        addPacket("4 Angle Pos", shooter.flapServo.getPosition());
        addPacket("5 Shooter Velocity", shooter.getShooterVelocity());
        addPacket("6 numRings", numRings);
        addPacket("7 shoot", shoot);
        addPacket("8 highGoal", highGoal);
        addPacket("9 Time", (System.currentTimeMillis() - startTime) / 1000);
        addPacket("Update Frequency (Hz)", 1 / timeDiff);

        drawGoal("black");
        drawRobot(drivetrain.x, drivetrain.y, drivetrain.theta, "green");
        drawRobot(t265.getCamX(), t265.getCamY(), t265.getCamTheta(), "red");
        drawRobot(x, y, theta, "black");
        if (numRings == 0) {
            drawRobot(targets[2][0], targets[2][1], targets[2][2], "red");
            drawLine(targets[2][0], targets[2][1], shootX[2], shootZ[2], "red");
        } else {
            String color;
            if (numRings == 3) {
                color = "yellow";
            } else if (numRings == 2) {
                color = "blue";
            } else {
                color = "green";
            }
            drawRobot(targets[numRings - 1][0], targets[numRings - 1][1], targets[numRings - 1][2], color);
            drawLine(targets[numRings - 1][0], targets[numRings - 1][1], shootX[numRings - 1], shootZ[numRings - 1], color);
        }
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
        double shooterX = x + 6.5 * Math.sin(theta);
        double shooterY = y - 6.5 * Math.cos(theta);

        // Calculate Robot Angle
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        /*double v = 4.5 * shooter.getShooterVelocity();
        double p = v * dy;
        double q = -v * dx;*/

        // Uses Angle Bisector for High Goal for more consistency
        if (targetNum == 3) {
            double d = 10;
            double a = Math.sqrt(Math.pow(dx + d/2, 2) + Math.pow(dy, 2));
            double b = Math.sqrt(Math.pow(dx - d/2, 2) + Math.pow(dy, 2));

            targetX += - d/2 + d * b / (a + b);
            dx = targetX - shooterX;
            drawLine(shooterX, shooterY, targetX, targetY, "blue");
        }

        // Calculate Shooter Angle
        double d = Math.sqrt(Math.pow(targetX - shooterX, 2) + Math.pow(targetY - shooterY, 2));
        double shooterAngle = -0.00333008 * d + 0.311806;

        /*double alignRobotAngle = Math.asin((dx * vx - dx * vy) / Math.sqrt(Math.pow(p, 2) + Math.pow(q, 2))) - Math.atan(q / p);
        double dz = targetZ - 8;
        double a = (386 * Math.pow(d, 2)) / (2 * Math.pow(v, 2));
        double b = d;
        double c = -dz - a;
        double quadraticRes = (-b - Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
        double shooterAngle = 0.27 * (Math.atan(quadraticRes) - 5 * Math.PI / 36) * 3 / Math.PI;*/

        double alignRobotAngle = Math.atan2(dy, dx) + 0.00268976 * d - 0.435135;
        double alignRobotX = shooterX - 6.5 * Math.sin(alignRobotAngle);
        double alignRobotY = shooterY + 6.5 * Math.cos(alignRobotAngle);

        return new double[] {alignRobotX, alignRobotY, alignRobotAngle, shooterAngle};
    }

    public void highGoalShoot() {
        if (!shoot) {
            shootDelay = 30;
//            shooter.setShooterVelocity(-2000);
            highGoal = true;
            initiateShoot();
        }
    }

    public void powerShotShoot() {
        if (!shoot) {
            shootDelay = 40;
//            shooter.setShooterVelocity(-900);
            highGoal = false;
            initiateShoot();
        }
    }

    public void initiateShoot() {
        shoot = true;
        numRings = 3;
        shootCounter = -1;
    }

    // set target point (default K values)
    public void setTargetPoint(double xtarget, double ytarget, double thetatarget) {
        double thetak = drivetrain.thetak;
        double xk = drivetrain.xk;
        double yk = drivetrain.yk;

        // Make Sure thetatarget is Between 0 and 2pi
        thetatarget = thetatarget % (Math.PI * 2);
        if (thetatarget < 0) {
            thetatarget += Math.PI * 2;
        }

        // Picking the Smaller Distance to Rotate
        double thetacontrol = 0;
        if (theta - thetatarget > Math.PI) {
            thetacontrol = theta - thetatarget - 2 * Math.PI;
        } else if (theta - thetatarget < (-Math.PI)) {
            thetacontrol = theta - thetatarget + 2 * Math.PI;
        } else {
            thetacontrol = theta - thetatarget;
        }

        log("setTargetPoint- " + xtarget + " " + " " + ytarget + " " + thetatarget);

        drivetrain.setGlobalControls(-xk * (x - xtarget), -yk * (y - ytarget), -thetak * (thetacontrol));
    }

    // set target point (custom K values)
    public void setTargetPoint(double xtarget, double ytarget, double thetatarget, double xK, double yK, double thetaK) {
        // Make Sure thetatarget is Between 0 and 2pi
        thetatarget = thetatarget % (Math.PI * 2);
        if (thetatarget < 0) {
            thetatarget += Math.PI * 2;
        }

        // Picking the Smaller Distance to Rotate
        double thetacontrol = 0;
        if (Math.abs(theta - thetatarget) > Math.PI) {
            thetacontrol = theta - thetatarget - 2 * Math.PI;
        } else {
            thetacontrol = theta - thetatarget;
        }

        log("setTargetPointK-" + xtarget + " " + " " + ytarget + " " + thetatarget);

        drivetrain.setGlobalControls(-xK * (x - xtarget), -yK * (y - ytarget), -thetaK * (thetacontrol));
    }

    // check if robot is at a certain point/angle (default tolerance)
    public boolean isAtPose(double targetx, double targety, double targettheta) {
        return isAtPose(targetx, targety, targettheta, xyTolerance, xyTolerance, thetaTolerance);
    }

    // check if robot is at a certain point/angle (custom tolerance)
    public boolean isAtPose(double targetx, double targety, double targettheta, double xtolerance, double ytolerance, double thetatolerance) {
        return (Math.abs(x - targetx) < xtolerance && Math.abs(y - targety) < ytolerance && Math.abs(theta - targettheta) < thetatolerance);
    }

    // draw on dashboard field
    public void drawRobot(double robotX, double robotY, double robotTheta, String color) {
        double r = 9 * Math.sqrt(2);
        double pi = Math.PI;
        double x = robotY - 72;
        double y = 72 - robotX;
        double theta = pi/2 + robotTheta;
        double[] ycoords = {r * Math.sin(pi/4 + theta) + y, r * Math.sin(3 * pi/4 + theta) + y, r * Math.sin(5 * pi/4 + theta) + y, r * Math.sin(7 * pi/4 + theta) + y};
        double[] xcoords = {r * Math.cos(pi/4 + theta) + x, r * Math.cos(3 * pi/4 + theta) + x, r * Math.cos(5 * pi/4 + theta) + x, r * Math.cos(7 * pi/4 + theta) + x};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
    }

    public void drawGoal(String color) {
        double[] xcoords = {72, 72, 78, 78};
        double[] ycoords = {-24, -48, -48, -24};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
    }

    public void drawLine(double x1, double y1, double x2, double y2, String color) {
        packet.fieldOverlay().setStroke(color).strokeLine(y1 - 72, 72 - x1, y2 - 72, 72 - x2);
    }

    // dashboard telemetry
    public void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    public void log(String message) {
        Log.w("robot-log", message);
    }
}
