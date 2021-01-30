package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.Pathing.Pose;

import java.util.Arrays;
import java.util.List;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@Config
@SuppressWarnings("FieldCanBeLocal") @SuppressLint("DefaultLocale")
public class Robot {

    // Robot Classes
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public WobbleArm wobbleArm;
    public Shooter shooter;
    public T265 t265;
    public Logger logger;

    private List<LynxModule> allHubs;
    private VoltageSensor battery;
    private boolean startVoltTooLow = false;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI / 35;
    public static double xKp = 0.6;
    public static double yKp = 0.6;
    public static double thetaKp = 6.0;
    public static double xKd = 0.05;
    public static double yKd = 0.05;
    public static double thetaKd = 0.4;
    private double odoWeight = 1;

    public static int highGoalDelay = 150;
    public static int psDelay = 200;
    private double[] target = {};
    private boolean flickHome = true;

    // State Variables
    private final boolean isAuto;
    private boolean firstLoop = true;
    private int cycleCounter = 0;
    public int numRings = 0;
    public boolean shoot = false;
    public boolean highGoal = false;
    public boolean vibrateMag = false;
    public boolean preShoot = false;

    public int lastTarget = -1;

    // Time and Delay Variables
    public double shootTime;
    public double shootDelay;
    public double startShootTime;
    public double vibrateTime;
    public double vibrateDelay = 100;
    public double xOffset = 0;

    public double feedHomeDelay = 100;
    public double feedHomeTime;
    public boolean waitFeed = false;

    // Motion Variables
    public double x, y, theta;
    private double prevX, prevY, prevTheta, vx, vy, w, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // Shooter Variables
    private final double[] shootXCor = {76.5, 84, 91.5, 108};
    private final double shootYCor = 150;

    // Powershot Debug Variables
//    public static double theta0 = 1.6534;
//    public static double theta1 = 1.5984;
//    public static double theta2 = 1.5244;
//    public static double[][] powerTargets = {
//            {85, 63, theta0, 0},
//            {87, 63, theta1, 0},
//            {89, 63, theta2, 0}
//    };
    public static double[] psShoot = new double[] {87, 63};

    public static double flap0 = 0.45;
    public static double flap1 = 0.46;
    public static double flap2 = 0.5;
    public static double[] flapPositions = {flap0, flap1, flap2};

    // OpMode Stuff
    private LinearOpMode op;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto) {
        drivetrain = new MecanumDrivetrain(op, x, y, theta);
        intake = new Intake(op, isAuto);
        shooter = new Shooter(op);
        wobbleArm = new WobbleArm(op, isAuto);
        logger = new Logger();
        try {
            t265 = new T265(op, x, y, theta); t265.startCam();
        } catch (Exception ex) {
            log("Camera error"); ex.printStackTrace();
            odoWeight = 1;
        }

        allHubs = op.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.op = op;
        this.isAuto = isAuto;

        battery = op.hardwareMap.voltageSensor.iterator().next();
        log("Battery Voltage: " + battery.getVoltage() + "v");
        if (battery.getVoltage() < 12.4) {
            startVoltTooLow = true;
        }
        drawGoal("black");
        drawRobot(x, y, theta, "black");
        sendPacket();
    }

    // Stop logger and t265
    public void stop() {
        logger.stopLogging();
        if (odoWeight != 1) {
            t265.stopCam();
        }
    }

    // Reset Odometry
    public void resetOdo(double x, double y, double theta) {
        drivetrain.resetOdo(x, y, theta);
        if (odoWeight != 1) {
            t265.setCameraPose(x, y, theta);
        }
    }

    public void update() {
        cycleCounter++;

        // Powershot Debug
        flapPositions = new double[] {flap2, flap1, flap0};

        // Track time after start
        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        // Pre-shoot tasks: Turn on flywheel, move robot to shooting position, mag up, start auto-feed once ready
        if (preShoot && !vibrateMag) {

            // Set flywheel velocity and shooting position based on what we want to shoot
            double[] target;
            int vThresh;
            if (highGoal) {
                shooter.flywheelHG();
                vThresh = Constants.HIGH_GOAL_VELOCITY - 50;

                double shootY = 63;
                if (isAuto){
                    shootY = 144 - Math.sqrt(Math.pow(88,2) - Math.pow(x - 108,2));
                }
                target = shootTargets(x, shootY, PI / 2, 3);
            } else {
                shooter.flywheelPS();
                if (!isAuto){
                    intake.sticksOut();
                }
                vThresh = Constants.POWERSHOT_VELOCITY - 40;
                target = shootTargets(psShoot[0], psShoot[1], PI / 2, 2);
                shooter.setFlapPos(flapPositions[2]);
            }

            // Turn off intake and put mag up
            if (shooter.magHome) {
                intake.intakeOff();
                shooter.magShoot();
                if (!isAuto) {
                    intake.sticksFourth();
                }
                log("Mag up");
            }

            // Move to shooting position
            if (!isAtPose(target[0], target[1], target[2], 1, 1, (isAuto && !highGoal) ? PI/40 : PI/35)) {
                setTargetPoint(target[0], target[1], target[2], 0.4, 0.4, 3);
                log("("+x+", "+y+", "+theta+") Moving to shoot position: " + Arrays.toString(target));
            }

            // Start auto-feed when mag is up, velocity is high enough, and robot is at position
            else if (!shooter.magHome && shooter.getVelocity() > vThresh && isAtPose(target[0], target[1], target[2])) {
                if (highGoal) {
                    shootDelay = highGoalDelay;
                } else {
                    shootDelay = psDelay;
                }
                shoot = true;
                numRings = 3;
                shootTime = System.currentTimeMillis();
                preShoot = false;
                log("Ready to shoot " + (highGoal ? "high goal" : "powershot") + ", velocity: " + shooter.getVelocity());
                log("Pre shoot time: " +  (System.currentTimeMillis() - startShootTime) + " ms");
            }
        }

        // Shoot tasks: change/maintain shooting position, auto feed rings
        if (shoot && numRings >= 0 && !shooter.magHome && !vibrateMag) {

            // Maintain/change robot alignment, set flap
            if (numRings > 0) {
                if (highGoal) {
                    target = shootTargets(3);
                } else {
//                    if (numRings == 3 || System.currentTimeMillis() - flickTime > flickDelay) {
//                        target = powerTargets[numRings - 1]; // flick move done flick // flick done move done flick
//                    }
                    target = shootTargets(2);
                    shooter.setFlapPos(flapPositions[numRings-1]);
                    lastTarget = numRings-1;
                }
                setTargetPoint(target[0], target[1], target[2], 0.4, 0.4, 4);
            }

            // Auto feed rings
            if (System.currentTimeMillis() - shootTime > shootDelay) {
                if (numRings > 0) {
                    // Shoot ring only if robot at position
                    if (isAtPose(target[0], target[1], target[2])) {
                        log("In shoot Velocity: " + shooter.getVelocity());

                        if (flickHome) {
                            shooter.feedMid();
                        } else {
                            shooter.feedHome();
                            numRings--;
                        }

                        if (numRings == 3 && !isAuto) {
                            intake.sticksOut();
                        }

                        flickHome = !flickHome;
                        log("Feed ring");
                    }
                } else {
                    shooter.flywheelOff();
                    shooter.magHome();
                    shoot = false;
                    log("Shoot done");
                    log("Total shoot time: " +  (System.currentTimeMillis() - startShootTime) + " ms");
                }
                shootTime = System.currentTimeMillis();
            }
        }

        // To help unstuck rings--> intake on, vibrate pos, intake rev, home pos, intake off
        if (vibrateMag && System.currentTimeMillis() - vibrateTime > vibrateDelay) {
            if (!intake.on && shooter.magHome) {
                intake.intakeOn();
            } else if (intake.forward && shooter.magHome) {
                shooter.magVibrate();
            } else if (intake.forward && shooter.magVibrate) {
                intake.intakeRev();
            } else if (intake.reverse && shooter.magVibrate) {
                shooter.magHome();
                intake.intakeOff();
                vibrateMag = false;
            }
            vibrateTime = System.currentTimeMillis();
        }

        // Wait for feed to go home before mag down
        if (waitFeed && System.currentTimeMillis() - feedHomeTime > feedHomeDelay) {
            shooter.magHome();
            waitFeed = false;
        }

        // Update Position
        drivetrain.updatePose();
        if (odoWeight != 1) {
            //t265.sendOdometryData(vx, vy);
            t265.updateCamPose();
        }

        // Calculate Motion Info
        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        if (odoWeight != 1) {
            x = odoWeight * drivetrain.x + (1 - odoWeight) * t265.getCamX();
            y = odoWeight * drivetrain.y + (1 - odoWeight) * t265.getCamY();
            theta = odoWeight * drivetrain.theta + (1 - odoWeight) * t265.getCamTheta();
        } else {
            x = drivetrain.x;
            y = drivetrain.y;
            theta = drivetrain.theta;
        }
        vx = (x - prevX) / timeDiff;
        vy = (y - prevY) / timeDiff;
        w = (theta - prevTheta) / timeDiff;
        ax = (vx - prevVx) / timeDiff;
        ay = (vy - prevVy) / timeDiff;
        a = (w - prevW) / timeDiff;

        // Remember Previous Motion Info
        prevX = x;
        prevY = y;
        prevTheta = theta;
        prevTime = curTime;
        prevVx = vx;
        prevVy = vy;
        prevW = w;

        // Log Data
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime, x, y, theta, vx, vy, w, ax, ay, a, numRings, shooter.magHome, shooter.feedHome, lastTarget);
        }

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("1 1***", "Starting Battery Voltage < 12.5!!!!");
        }
        addPacket("1 X", String.format("%.5f", x));
        addPacket("2 Y", String.format("%.5f", y));
        addPacket("3 Theta", String.format("%.5f", theta));
        addPacket("4 Shooter Velocity", shooter.getVelocity());
        addPacket("5 numRings", numRings);
        addPacket("6 shoot", shoot  + " " + preShoot + " " + highGoal);
        addPacket("7 Time", (System.currentTimeMillis() - startTime) / 1000);
        addPacket("8 Update Frequency (Hz)", 1 / timeDiff);

        // Dashboard Drawings
        drawGoal("black");
        drawRobot(drivetrain.x, drivetrain.y, drivetrain.theta, "green");
        if (odoWeight != 1) {
            drawRobot(t265.getCamX(), t265.getCamY(), t265.getCamTheta(), "red");
        }
        drawRobot(x, y, theta, "black");
        sendPacket();

        // Clear bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    // Set variables for high goal shoot
    public void highGoalShoot() {
        if (!preShoot) {
            flickHome = true;
            preShoot = true;
            highGoal = true;
            startShootTime = System.currentTimeMillis();
            log("High goal shoot initiated");
        }
    }

    // Set variables for powershot shoot
    public void powerShotShoot() {
        if (!preShoot) {
            flickHome = true;
            preShoot = true;
            highGoal = false;
            startShootTime = System.currentTimeMillis();
            log("Powershot shoot initiated");
        }
    }

    // Cancel shoot sequence
    public void cancelShoot() {
        preShoot = false;
        shoot = false;
        numRings = 0;
        shooter.flywheelOff();
        intake.sticksOut();
        if (shooter.feedHome) {
            shooter.magHome();
        } else {
            shooter.feedHome();
            waitFeed = true;
            feedHomeTime = System.currentTimeMillis();
        }
        log("Shoot cancelled");
    }

    // Calculate robot pose for auto aim
    // left ps = 0, middle ps = 1, right ps = 2, high goal = 3
    public double[] shootTargets(int targetNum) {
        return shootTargets(x, y, theta, targetNum);
    }

    public double[] shootTargets(double shootX, double shootY, double shootTheta, int targetNum) {
        /*  power1- (76.5,144,24)
            power2- (84,144,24)
            power3- (91.5,144,24)
            high goal- (108,144,35.5) */

        if (highGoal) lastTarget = targetNum;

        double targetX = shootXCor[targetNum];
        double targetY = shootYCor;

        double shooterX = shootX + 6.5 * Math.sin(shootTheta);
        double shooterY = shootY - 6.5 * Math.cos(shootTheta);
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;

        // Uses Angle Bisector for High Goal for more consistency
        if (targetNum == 3) {
            double d = 8;
            double a = Math.sqrt(Math.pow(dx + d/2, 2) + Math.pow(dy, 2));
            double b = Math.sqrt(Math.pow(dx - d/2, 2) + Math.pow(dy, 2));

            targetX += - d/2 + d * b / (a + b);
            dx = targetX - shooterX;
            drawLine(shooterX, shooterY, targetX, targetY, "blue");
        }

        // Calculate Robot Angle
        double d = Math.sqrt(Math.pow(targetX - shooterX, 2) + Math.pow(targetY - shooterY, 2));
        double alignRobotAngle = Math.atan2(dy, dx) + 0.0013 * d - 0.2300;
        double alignRobotX = shooterX - 6.5 * Math.sin(alignRobotAngle) + xOffset;
        double alignRobotY = shooterY + 6.5 * Math.cos(alignRobotAngle);

        return new double[] {alignRobotX, alignRobotY, alignRobotAngle};
    }

    // Set target point (default K values)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget) {
        setTargetPoint(xTarget, yTarget, thetaTarget, xKp, yKp, thetaKp);
    }

    // Set target point (using pose)
    public void setTargetPoint(Pose pose, double placeholder) {
        setTargetPoint(pose.getX(), pose.getY(), pose.getTheta());
    }

    // Set target point (custom Kp values)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget, double xKp, double yKp, double thetaKp) {
        // Make Sure thetaTarget is Between 0 and 2pi
        thetaTarget = thetaTarget % (PI * 2);
        if (thetaTarget < 0) {
            thetaTarget += PI * 2;
        }

        // Picking the Smaller Distance to Rotate
        double thetaControl;
        if (Math.abs(theta - thetaTarget) > PI) {
            thetaControl = theta - thetaTarget - 2 * PI;
        } else {
            thetaControl = theta - thetaTarget;
        }

        drivetrain.setGlobalControls(xKp * (xTarget - x) + xKd * (-vx), yKp * (yTarget - y) + yKd * (-vy), thetaKp * (-thetaControl) + thetaKd * (-w));

    }

    // Set target point (using pose, custom Kp values)
    public void setTargetPoint(Pose pose, double xKp, double yKp, double thetaKp) {
        setTargetPoint(pose.getX(), pose.getY(), pose.getTheta(), xKp, yKp, thetaKp);
    }

    // Set target point (velocity specification)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget, double vxTarget, double vyTarget, double wTarget, double dummy) {
        setTargetPoint(xTarget, yTarget, thetaTarget, vxTarget, vyTarget, wTarget, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
    }

    // Set target point (using pose, velocity specification)
    public void setTargetPoint(Pose pose) {
        setTargetPoint(pose.getX(), pose.getY(), pose.getTheta(), pose.getVx(), pose.getVy(), pose.getW(), xKp, yKp, thetaKp, xKd, yKd, thetaKd);
    }

    public void setTargetPoint(Pose pose, double theta, double w) {
        setTargetPoint(pose.getX(), pose.getY(), theta, pose.getVx(), pose.getVy(), w, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
    }

    // Set target point (velocity specification, custom Kp and Kv values)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget, double vxTarget, double vyTarget, double wTarget, double xKp, double yKp, double thetaKp, double xKd, double yKd, double thetaKd) {
        // Make Sure thetaTarget is Between 0 and 2pi
        thetaTarget = thetaTarget % (PI * 2);
        if (thetaTarget < 0) {
            thetaTarget += PI * 2;
        }

        // Picking the Smaller Distance to Rotate
        double thetaControl;
        if (Math.abs(theta - thetaTarget) > PI) {
            thetaControl = theta - thetaTarget - 2 * PI;
        } else {
            thetaControl = theta - thetaTarget;
        }

        drivetrain.setGlobalControls(xKp * (xTarget - x) + xKd * (vxTarget - vx), yKp * (yTarget - y) + yKd * (vyTarget - vy), thetaKp * (-thetaControl) + thetaKd * (wTarget - w));
    }

    // Set target point (using pose, velocity specification, custom Kp and Kv values)
    public void setTargetPoint(Pose pose, double xKp, double yKp, double thetaKp, double xKd, double yKd, double thetaKd) {
        setTargetPoint(pose.getX(), pose.getY(), pose.getTheta(), pose.getVx(), pose.getVy(), pose.getW(), xKp, yKp, thetaKp, xKd, yKd, thetaKd);
    }

    // Check if robot is at a certain point/angle (default tolerance)
    public boolean isAtPose(double targetX, double targetY, double targetTheta) {
        return isAtPose(targetX, targetY, targetTheta, xyTolerance, xyTolerance, thetaTolerance);
    }

    // Check if robot is at a certain point/angle (custom tolerance)
    public boolean isAtPose(double targetX, double targetY, double targetTheta, double xTolerance, double yTolerance, double thetaTolerance) {
        return (Math.abs(x - targetX) < xTolerance && Math.abs(y - targetY) < yTolerance && Math.abs(theta - targetTheta) < thetaTolerance);
    }

    // Logging
    public static void log(String message) {
        Log.w("robot-log", message);
    }
}
