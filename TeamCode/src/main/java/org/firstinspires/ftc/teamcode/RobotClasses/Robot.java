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
import org.firstinspires.ftc.teamcode.Pathing.Target;

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

    private ElapsedTime profiler;
    private List<LynxModule> allHubs;
    private VoltageSensor battery;
    private boolean startVoltTooLow = false;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final int sensorUpdatePeriod = 15;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI/35;

    private double odoCovariance = 1;
    private boolean useT265 = false;

    // State Variables
    private final boolean isAuto;
    private boolean firstLoop = true;
    private int loopCounter = 0;
    public int numRings = 0;
    public boolean highGoal = false;
    public boolean preShoot = false;
    public boolean shoot = false;
    public boolean sweepRings = false;
    public int lastTarget = -1;

    public int cycles = 0;
    public double cycleTotal;
    public double lastCycleTime;

    // Time and Delay Variables
    public double shootTime;
    public double shootDelay;
    public double startShootTime;
    public static int highGoalDelay = 250;
    public static int psDelay = 450;
    public static double flapDelay = 250;
    public double flickTime;

    // Motion Variables
    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // Shooter Variables
    private final double[] shootXCor = {76.5, 84, 91.5, 108};
    private final double shootYCor = 150;
    private double[] target = {};

    public double shootYOverride = 0;
    public int numRingsPreset = 3;
    public double thetaOffset = 0;

    // Powershot Debug Variables
    public final double[] psShootPos = new double[] {111, 63};
    public static double theta0 = 1.9283;
    public static double theta1 = 1.8157;
    public static double theta2 = 1.7392;
    public static double[] thetaPositions = {theta2, theta1, theta0};

    public ArrayList<Ring> ringPos = new ArrayList<>();

    // OpMode Stuff
    private LinearOpMode op;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto) {
        drivetrain = new MecanumDrivetrain(op, x, y, theta);
        this.x = x;
        this.y = y;
        this.theta = theta;

        intake = new Intake(op, isAuto);
        shooter = new Shooter(op);
        wobbleArm = new WobbleArm(op);
        logger = new Logger();
        try {
            t265 = new T265(op, x, y, theta);
            t265.startCam();
        } catch (Exception ex) {
            log("Camera error");
            ex.printStackTrace();
            useT265 = false;
        }

        profiler = new ElapsedTime();

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
        drawGoals("black");
        drawRobot(this, "black");
        sendPacket();
    }

    // Stop logger and t265
    public void stop() {
        logger.stopLogging();
        if (useT265) {
            t265.stopCam();
        }
    }

    // Reset Odometry
    public void resetOdo(double x, double y, double theta) {
        drivetrain.resetOdo(x, y, theta);
        if (useT265) {
            t265.setCameraPose(x, y, theta);
        }
    }

    public void update() {
        loopCounter++;
        profiler.reset();

        // Powershot Debug
        thetaPositions = new double[] {theta2, theta1, theta0};

        // Track time after start
        if (firstLoop) {
            startTime = System.currentTimeMillis();
            lastCycleTime = System.currentTimeMillis();
            firstLoop = false;
        }

        if (!isAuto && !preShoot && !shoot && !shooter.sensorBroken && loopCounter % sensorUpdatePeriod == 0) {
            numRings = shooter.getNumRings();
        }

        profile(1);

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
                    if (shootYOverride != 0) {
                        shootY = shootYOverride;
                    }
                }
                target = shootTargets(x, shootY, PI/2, 3);
            } else {
                shooter.flywheelPS();
                vThresh = Constants.POWERSHOT_VELOCITY - 40;
                target = new double[] {psShootPos[0], psShootPos[1], thetaPositions[2]};
            }

            // Turn off intake and put mag up
            if (shooter.magHome) {
                intake.off();
                shooter.magShoot();
                if (!isAuto && numRingsPreset == 3) {
                    intake.sticksOut();
                }
                log("Mag up");
            }

            // Move to shooting position
            if (!isAtPose(target[0], target[1], target[2], 1, 1, PI/35) || Math.abs(vx) + Math.abs(vy) > 1.5 || Math.abs(w) > 0.1) {
                setTargetPoint(target[0], target[1], target[2]);
                log("(" + round(x) + ", " + round(y) + ", " + round(theta) + ") Moving to shoot position: " + Arrays.toString(target));
            }

            // Start auto-feed when mag is up, velocity is high enough, and robot is at position
            else if (!shooter.magHome && shooter.getVelocity() > vThresh) {
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

        profile(2);

        // Shoot tasks: change/maintain shooting position, auto feed rings
        if (shoot && numRings >= 0 && !shooter.magHome) {

            // Maintain/change robot alignment, set flap
            if (numRings > 0) {
                if (highGoal) {
                    target = shootTargets(3);
                    lastTarget = 3;
                } else if (numRings == 3 || System.currentTimeMillis() - flickTime > flapDelay) {
                    target = new double[] {psShootPos[0], psShootPos[1], thetaPositions[numRings - 1]};
                    lastTarget = 3 - numRings;
                }
                setTargetPoint(target[0], target[1], target[2]);
            }

            // Auto feed rings
            if (System.currentTimeMillis() - shootTime > shootDelay) {
                if (numRings > 0) {
                    // Shoot ring only if robot at position and velocity low enough
                    if ((highGoal && isAtPose(target[0], target[1], target[2], 1, 1, PI/75)) ||
                            (!highGoal && isAtPose(target[0], target[1], target[2], 1, 1, PI/150)
                            && Math.abs(vx) + Math.abs(vy) < 1.5 && Math.abs(w) < 0.1)) {
                        log("In shoot Velocity: " + shooter.getVelocity());
                        log("Drivetrain Velocities: " + round(vx) + " " + round(vy) + " " + round(w));
                        if (!highGoal) {
                            log("theta: " + theta);
                        }

                        if (shooter.feedHome) {
                            shooter.feedTop();
                        } else {
                            shooter.feedHome();
                        }

                        if (numRings == 3) {
                            log("Feed ring 1");
                        } else if (numRings == 2) {
                            log("Feed ring 2");
                        } else if (numRings == 1) {
                            log("Feed ring 3");

                        }

                        numRings--;
                        flickTime = System.currentTimeMillis();
                    } else {
                        log("(" + round(x) + ", " + round(y) + ", " + round(theta) + ") Moving to shoot position: " + Arrays.toString(target));
                    }
                } else {
                    shooter.flywheelOff();
                    shooter.magHome();
                    shoot = false;

                    if (!isAuto) {
                        sweepSticks();
                    }

                    log("Shoot done");
                    log("Total shoot time: " +  (System.currentTimeMillis() - startShootTime) + " ms");
                    double cycleTime = (System.currentTimeMillis() - lastCycleTime) / 1000;
                    cycleTotal += cycleTime;
                    cycles++;
                    Log.w("cycle-log", "Cycle " + cycles + ": " + cycleTime + "s");
                    lastCycleTime = System.currentTimeMillis();
                }
                shootTime = System.currentTimeMillis();
            }
        }

        if (sweepRings && System.currentTimeMillis() - shootTime > 1500) {
            intake.sticksCollect();
            sweepRings = false;
        }

        profile(3);

        // Update Position
        drivetrain.updatePose();
        if (odoCovariance != 1) {
            //t265.sendOdometryData(vx, vy);
            t265.updateCamPose();
        }
        intake.updateSticks();

        profile(4);

        // Calculate Motion Info
        double curTime = System.currentTimeMillis() / 1000.0;
        double timeDiff = curTime - prevTime;
        if (useT265) {
            x = odoCovariance * drivetrain.x + (1 - odoCovariance) * t265.getCamX();
            y = odoCovariance * drivetrain.y + (1 - odoCovariance) * t265.getCamY();
            theta = odoCovariance * drivetrain.theta + (1 - odoCovariance) * t265.getCamTheta();
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
        prevX = x; prevY = y;
        prevTheta = theta;
        prevTime = curTime;
        prevVx = vx; prevVy = vy;
        prevW = w;

        profile(5);

        // Log Data
        if (loopCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis() - startTime, x, y, theta, vx, vy, w, ax, ay, a, numRings, shooter.magHome, shooter.feedHome, lastTarget, cycles, cycleTotal / cycles);
        }

        profile(6);

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("0", "Starting Battery Voltage < 12.4!!!!");
        }
        if (shooter.sensorBroken) {
            addPacket("00", "Distance Sensor Broken!!!!");
        }
        addPacket("1 X", round(x));
        addPacket("2 Y", round(y));
        addPacket("3 Theta", round(theta));
        addPacket("4 Shooter Velocity", shooter.getVelocity());
        addPacket("5 numRings", numRings);
        addPacket("6 shoot", shoot  + " " + preShoot + " " + highGoal);
        addPacket("7 Time", (System.currentTimeMillis() - startTime) / 1000);
        addPacket("8 Update Frequency (Hz)", 1 / timeDiff);
        if (!isAuto) {
            addPacket("Cycle Time", (System.currentTimeMillis() - lastCycleTime) / 1000);
            addPacket("Average Cycle Time", cycleTotal / cycles);
            addPacket("Cycles", cycles);
        }

        // Dashboard Drawings
        drawGoals("black");
        /*drawRobot(drivetrain.x, drivetrain.y, drivetrain.theta, "green");
        if (useT265) {
            drawRobot(t265.getCamX(), t265.getCamY(), t265.getCamTheta(), t265.confidenceColor());
        }*/
        drawRobot(this, "black");
        for (Ring ring : ringPos) {
            drawRing(ring);
        }
//        log("rings: " + ringPos);
        sendPacket();

        profile(7);

        // Clear bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        profile(8);
    }

    // Set variables for high goal shoot
    public void highGoalShoot(int numRings) {
        if (!preShoot && !shoot) {
            preShoot = true;
            highGoal = true;
            numRingsPreset = numRings;
            if (numRings != 3) {
                log("Shooting with " + numRings + " rings");
            }
            if (shootYOverride != 0) {
                log("Shooting at y = " + shootYOverride);
            }
            startShootTime = System.currentTimeMillis();
            log("High goal shoot initiated");
        }
    }

    public void highGoalShoot() {
        highGoalShoot(!shooter.sensorBroken ? shooter.getNumRings() : 3);
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
        sweepRings = false;
        numRings = 0;
        numRingsPreset = 0;
        shootYOverride = 0;
        shooter.flywheelOff();
        intake.sticksCollect();
        shooter.magHome();
        log("Shoot cancelled");
    }

    public void sweepSticks() {
        if (!preShoot && !shoot && !sweepRings) {
            intake.sticksSweep();
            sweepRings = true;
            shootTime = System.currentTimeMillis();
        }
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
        double alignRobotAngle = Math.atan2(dy, dx) + 0.0013 * d - 0.2300 - thetaOffset;
        double alignRobotX = shooterX - 6.5 * Math.sin(alignRobotAngle);
        double alignRobotY = shooterY + 6.5 * Math.cos(alignRobotAngle);

        return new double[] {alignRobotX, alignRobotY, alignRobotAngle};
    }

    // Set target point (velocity specification, custom Kp and Kv values)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget, double vxTarget, double vyTarget, double wTarget, double xKp, double yKp, double thetaKp, double xKd, double yKd, double thetaKd) {
        // Make Sure thetaTarget is Between 0 and 2pi
        thetaTarget = thetaTarget % (2*PI);
        if (thetaTarget < 0) {
            thetaTarget += 2*PI;
        }

        // Picking the Smaller Distance to Rotate
        double thetaControl;
        if (Math.abs(theta - thetaTarget) > PI) {
            thetaControl = theta - thetaTarget - 2*PI;
        } else {
            thetaControl = theta - thetaTarget;
        }

         // log("target theta: " + thetaTarget);

        drawRobot(xTarget, yTarget, thetaTarget, "blue");

        drivetrain.setGlobalControls(xKp * (xTarget - x) + xKd * (vxTarget - vx), yKp * (yTarget - y) + yKd * (vyTarget - vy), thetaKp * (-thetaControl) + thetaKd * (wTarget - w));
    }

    // Set target point (default Kp and Kv gains)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget) {
        setTargetPoint(xTarget, yTarget, thetaTarget, 0, 0, 0, MecanumDrivetrain.xKp, MecanumDrivetrain.yKp, MecanumDrivetrain.thetaKp, MecanumDrivetrain.xKd, MecanumDrivetrain.yKd, MecanumDrivetrain.thetaKd);
    }

    // Set target point (using pose, velocity specification, default Kp and Kv gains)
    public void setTargetPoint(Pose pose) {
        setTargetPoint(pose.x, pose.y, pose.theta, pose.vx, pose.vy, pose.w, MecanumDrivetrain.xKp, MecanumDrivetrain.yKp, MecanumDrivetrain.thetaKp, MecanumDrivetrain.xKd, MecanumDrivetrain.yKd, MecanumDrivetrain.thetaKd);
    }

    // Set target point (using pose, custom theta and omega, default Kp and Kv gains)
    public void setTargetPoint(Pose pose, double theta, double w) {
        setTargetPoint(pose.x, pose.y, theta, pose.vx, pose.vy, w, MecanumDrivetrain.xKp, MecanumDrivetrain.yKp, MecanumDrivetrain.thetaKp, MecanumDrivetrain.xKd, MecanumDrivetrain.yKd, MecanumDrivetrain.thetaKd);
    }

    // Set target point (using target object)
    public void setTargetPoint(Target target) {
        Pose pose = target.getPose();
        setTargetPoint(pose.x, pose.y, pose.theta, pose.vx, pose.vy, pose.w, target.xKp(), target.yKp(), target.thetaKp(), target.xKd(), target.yKd(), target.thetaKd());
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

    private void profile(int num) {
        //Log.w("profiler", num + ": " + profiler.milliseconds());
    }

    @SuppressLint("DefaultLocale")
    public static double round(double num) {
        return Double.parseDouble(String.format("%.5f", num));
    }
}
