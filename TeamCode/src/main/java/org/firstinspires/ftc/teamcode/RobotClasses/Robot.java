package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.Pathing.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@Config
@SuppressWarnings("FieldCanBeLocal")
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
    private final double thetaTolerance = PI/35;

    private double odoWeight = 1;

    // State Variables
    private final boolean isAuto;
    private boolean firstLoop = true;
    private int cycleCounter = 0;
    public int numRings = 0;
    public boolean shoot = false;
    public boolean highGoal = false;
    public boolean preShoot = false;
    private boolean waitFeed = false;
    public int lastTarget = -1;

    // Time and Delay Variables
    public double shootTime;
    public double shootDelay;
    public double startShootTime;
    public double feedHomeTime;
    public double feedHomeDelay = 100;
    public static int highGoalDelay = 250;
    public static int psDelay = 250;

    // Motion Variables
    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // Shooter Variables
    private final double[] shootXCor = {76.5, 84, 91.5, 108};
    private final double shootYCor = 150;

    public double shootYOverride = 0;
    public int numRingsPreset = 3;
    public double xOffset = 0;

    // Powershot Debug Variables
    public final double[] psShootPos = new double[] {87, 63};
    public static double flap0 = 0.45;
    public static double flap1 = 0.47;
    public static double flap2 = 0.5;
    public static double[] flapPositions = {flap0, flap1, flap2};

    public ArrayList<Ring> ringPos = new ArrayList<>();

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
        drawRobot(this, "black");
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

        ElapsedTime t = new ElapsedTime();

        cycleCounter++;

        // Powershot Debug
        flapPositions = new double[] {flap2, flap1, flap0};

        // Track time after start
        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        Log.w("time1", t.milliseconds()+"");

        // Pre-shoot tasks: Turn on flywheel, move robot to shooting position, mag up, start auto-feed once ready
        if (preShoot) {

            // Set flywheel velocity and shooting position based on what we want to shoot
            double[] target;
            int vThresh;
            if (highGoal) {
                shooter.flywheelHG();
                vThresh = Constants.HIGH_GOAL_VELOCITY - 50;

                double shootY = 63;
                if (isAuto) {
                    shootY = 144 - Math.sqrt(Math.pow(88,2) - Math.pow(x - 108,2));
                    if (shootYOverride != 0) {
                        shootY = shootYOverride;
                    }
                }
                target = shootTargets(x, shootY, PI/2, 3);
            } else {
                shooter.flywheelPS();
                if (!isAuto){
                    intake.sticksOut();
                }
                vThresh = Constants.POWERSHOT_VELOCITY - 40;
                target = shootTargets(psShootPos[0], psShootPos[1], PI/2, 2);
                shooter.setFlapPosition(flapPositions[2]);
            }

            // Turn off intake and put mag up
            if (shooter.magHome) {
                intake.off();
                shooter.magShoot();
                if (!isAuto) {
                    intake.sticksFourth();
                }
                log("Mag up");
            }

            // Move to shooting position
            if (!isAtPose(target[0], target[1], target[2], 0.5, 0.5, PI/35)) {
                setTargetPoint(target[0], target[1], target[2]);
                log("(" + round(x) + ", " + round(y) + ", " + round(theta) + ") Moving to shoot position: " + Arrays.toString(target));
            }

            // Start auto-feed when mag is up, velocity is high enough, and robot is at position
            else if (!shooter.magHome && shooter.getVelocity() > vThresh && isAtPose(target[0], target[1], target[2], 0.5, 0.5, PI/35)) {
                if (highGoal) {
                    shootDelay = highGoalDelay;
                } else {
                    shootDelay = psDelay;
                }
                shoot = true;
                numRings = numRingsPreset;
                shootYOverride = 0;
                shootTime = System.currentTimeMillis();
                preShoot = false;
                log("Ready to shoot " + (highGoal ? "high goal" : "powershot") + ", velocity: " + shooter.getVelocity());
                log("Pre shoot time: " +  (System.currentTimeMillis() - startShootTime) + " ms");
            }
        }

        // Shoot tasks: change/maintain shooting position, auto feed rings
        if (shoot && numRings >= 0 && !shooter.magHome) {

            // Maintain/change robot alignment, set flap
            double[] target = {};
            if (numRings > 0) {
                if (highGoal) {
                    target = shootTargets(3);
                    lastTarget = 3;
                } else {
                    target = shootTargets(2);
                    shooter.setFlapPosition(flapPositions[numRings-1]);
                    lastTarget = numRings-1;
                }
                setTargetPoint(target[0], target[1], target[2]);
            }

            // Auto feed rings
            if (System.currentTimeMillis() - shootTime > shootDelay) {
                if (numRings > 0) {
                    // Shoot ring only if robot at position
                    if (isAtPose(target[0], target[1], target[2])) {
                        log("In shoot Velocity: " + shooter.getVelocity());

                        if (numRings == 3) {
                            shooter.feedTop();
                            if (!isAuto) {
                                intake.sticksOut();
                            }
                            log("Feed ring 1");
                        } else if (numRings == 2) {
                            shooter.feedHome();
                            log("Feed ring 2");
                        } else if (numRings == 1) {
                            shooter.feedTop();
                            log("Feed ring 3");
                        }
                        numRings--;
                    }
                } else {
                    shooter.flywheelOff();
                    shooter.feedHome();
                    shoot = false;
                    waitFeed = true;
                    feedHomeTime = System.currentTimeMillis();
                    log("Shoot done");
                    log("Total shoot time: " +  (System.currentTimeMillis() - startShootTime) + " ms");
                }
                shootTime = System.currentTimeMillis();
            }
        }

        // Wait for feed to go home before mag down
        if (waitFeed && System.currentTimeMillis() - feedHomeTime > feedHomeDelay) {
            shooter.magHome();
            waitFeed = false;
        }

        Log.w("time2", t.milliseconds()+"");

        // Update Position
        drivetrain.updatePose();
        if (odoWeight != 1) {
            //t265.sendOdometryData(vx, vy);
            t265.updateCamPose();
        }

        Log.w("time3", t.milliseconds()+"");

        // Calculate Motion Info
        double curTime = System.currentTimeMillis() / 1000.0;
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

        Log.w("time4", t.milliseconds()+"");

        // Log Data
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime, x, y, theta, vx, vy, w, ax, ay, a, numRings, shooter.magHome, shooter.feedHome, lastTarget);
        }

        Log.w("time5", t.milliseconds()+"");

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("1 1***", "Starting Battery Voltage < 12.5!!!!");
        }
        addPacket("1 X", round(x));
        addPacket("2 Y", round(y));
        addPacket("3 Theta", round(theta));
//        addPacket("4 Shooter Velocity", shooter.getVelocity());
        addPacket("5 numRings", numRings);
        addPacket("6 shoot", shoot  + " " + preShoot + " " + highGoal);
        addPacket("7 Time", (System.currentTimeMillis() - startTime) / 1000);
        addPacket("8 Update Frequency (Hz)", 1 / timeDiff);
        addPacket("diff", timeDiff);

        // Dashboard Drawings
        drawGoal("black");
        drawRobot(drivetrain.x, drivetrain.y, drivetrain.theta, "green");
        if (odoWeight != 1) {
            drawRobot(t265.getCamX(), t265.getCamY(), t265.getCamTheta(), "red");
        }
        drawRobot(this, "black");
        for (Ring ring : ringPos) {
            drawRing(ring);
        }
        sendPacket();

        Log.w("time6", t.milliseconds()+"");
        t.reset();

        // Clear bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    // Set variables for high goal shoot
    public void highGoalShoot(int numRings) {
        if (!preShoot && !shoot) {
            preShoot = true;
            highGoal = true;
            numRingsPreset = numRings;
            if (numRings != 3) log("Shooting with " + numRings + " rings");
            startShootTime = System.currentTimeMillis();
            log("High goal shoot initiated");
        }
    }

    public void highGoalShoot() {
        highGoalShoot(3);
    }

    // Set variables for powershot shoot
    public void powerShotShoot() {
        if (!preShoot && !shoot) {
            preShoot = true;
            highGoal = false;
            numRingsPreset = 3;
            startShootTime = System.currentTimeMillis();
            log("Powershot shoot initiated");
        }
    }

    // Cancel shoot sequence
    public void cancelShoot() {
        preShoot = false;
        shoot = false;
        numRings = 0;
        numRingsPreset = 0;
        shootYOverride = 0;
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

        drawRobot(xTarget, yTarget, thetaTarget, "blue");

        drivetrain.setGlobalControls(xKp * (xTarget - x) + xKd * (vxTarget - vx), yKp * (yTarget - y) + yKd * (vyTarget - vy), thetaKp * (-thetaControl) + thetaKd * (wTarget - w));
    }

    // Set target point (default Kp and Kv gains)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget) {
        setTargetPoint(xTarget, yTarget, thetaTarget, 0, 0, 0, drivetrain.xKp, drivetrain.yKp, drivetrain.thetaKp, drivetrain.xKd, drivetrain.yKd, drivetrain.thetaKd);
    }

    // Set target point (velocity specification, default Kp and Kv gains)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget, double vxTarget, double vyTarget, double wTarget) {
        setTargetPoint(xTarget, yTarget, thetaTarget, vxTarget, vyTarget, wTarget, drivetrain.xKp, drivetrain.yKp, drivetrain.thetaKp, drivetrain.xKd, drivetrain.yKd, drivetrain.thetaKd);
    }

    // Set target point (using pose, velocity specification, default Kp and Kv gains)
    public void setTargetPoint(Pose pose) {
        setTargetPoint(pose.getX(), pose.getY(), pose.getTheta(), pose.getVx(), pose.getVy(), pose.getW(), drivetrain.xKp, drivetrain.yKp, drivetrain.thetaKp, drivetrain.xKd, drivetrain.yKd, drivetrain.thetaKd);
    }

    // Set target point (using pose, custom theta and omega, default Kp and Kv gains)
    public void setTargetPoint(Pose pose, double theta, double w) {
        setTargetPoint(pose.getX(), pose.getY(), theta, pose.getVx(), pose.getVy(), w, drivetrain.xKp, drivetrain.yKp, drivetrain.thetaKp, drivetrain.xKd, drivetrain.yKd, drivetrain.thetaKd);
    }

    public void setTargetPoint(Pose pose, double theta, double w, double xyKd) {
        setTargetPoint(pose.getX(), pose.getY(), theta, pose.getVx(), pose.getVy(), w, drivetrain.xKp, drivetrain.yKp, drivetrain.thetaKp, xyKd, xyKd, drivetrain.thetaKd);
    }

    public void setTargetPoint(Pose pose, double xyKp) {
        setTargetPoint(pose.getX(), pose.getY(), theta, pose.getVx(), pose.getVy(), w, xyKp, xyKp, drivetrain.thetaKp, drivetrain.xKd, drivetrain.yKd, drivetrain.thetaKd);
    }

    // Set target point (using pose, velocity specification, custom Kp and Kv gains)
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

    @SuppressLint("DefaultLocale")
    public double round(double num) {
        return Double.parseDouble(String.format("%.5f", num));
    }
}
