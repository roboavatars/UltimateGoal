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
import java.util.List;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawLine;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRing;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static org.firstinspires.ftc.teamcode.RobotClasses.Robot.TurretMode.HIGH_GOAL;
import static org.firstinspires.ftc.teamcode.RobotClasses.Robot.TurretMode.MID_GOAL;
import static org.firstinspires.ftc.teamcode.RobotClasses.Robot.TurretMode.NONE;
import static org.firstinspires.ftc.teamcode.RobotClasses.Robot.TurretMode.PS_C;
import static org.firstinspires.ftc.teamcode.RobotClasses.Robot.TurretMode.PS_L;
import static org.firstinspires.ftc.teamcode.RobotClasses.Robot.TurretMode.PS_R;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class Robot {

    // Robot Classes
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public WobbleArm wobbleArm;
    public Shooter shooter;
    public Logger logger;

    private ElapsedTime profiler;
    private List<LynxModule> allHubs;
    private VoltageSensor battery;
    private boolean startVoltTooLow = false;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI/35;
    private final double turretTolerance = PI/50;

    // State Variables
    private final boolean isAuto;
    private final boolean isRed;
    private boolean firstLoop = true;
    private int loopCounter = 0;
    public int numRings = 0;
    public boolean highGoal = false;
    public boolean preShoot = false;
    public boolean preShootOverride = false;
    public boolean shootOverride = false;
    public boolean shoot = false;
    public int lastTarget = -1;

    public int cycles = 0;
    public double cycleTotal;
    public double lastCycleTime;
    public double longestCycle = 0;

    // Time and Delay Variables
    public double curTime;
    public double shootTime;
    public double startShootTime;
    public double flickTime;
    public double shootDelay;
    private int vThresh;
    public static double flickTimeBackup = 1000;
    public static int highGoalDelay = 150;
    public static int psDelay = 300;
    public static double flickDelay = 100;

    // Motion Variables
    public double x, y, theta, vx, vy, w, turretGlobalTheta;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // Shooter Variables
    public double hgDist, mgDist;
    private final double[] shootXCorR = {76.5, 84, 91.5, 108};
    private final double[] shootXCorB = {67.5, 60, 52.5, 36};
    private final double shootYCor = 150;
    private double shootTargetTheta;
    public boolean turretReset;

    public double shootYOverride = 0;
    public double shootYOffset = 0;
    public double flapPos = Constants.FLAP_DOWN_POS;
    public double flapOverride = 0;
    public int numRingsPreset = 3;
    public double thetaOffset = 0;

    private double lockX, lockY;
    private TurretMode turretMode = NONE;

    public enum TurretMode {PS_L, PS_C, PS_R, HIGH_GOAL, MID_GOAL, NONE}

    // Powershot Debug Variables
    public static double theta0R = 1.671;
    public static double theta1R = 1.590;
    public static double theta2R = 1.498;
    public static double[] thetaPositionsR = {theta0R, theta1R, theta2R};
    public static double theta0B = PI - theta0R;
    public static double theta1B = PI - theta1R;
    public static double theta2B = PI - theta2R;
    public static double[] thetaPositionsB = {theta0B, theta1B, theta2B};

    // Ring State Variables
    public ArrayList<Ring> shotRings = new ArrayList<>();
    public ArrayList<Ring> ringPos = new ArrayList<>();

    // OpMode Stuff
    private LinearOpMode op;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, double turretTheta, boolean isAuto, boolean isRed) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.op = op;
        this.isAuto = isAuto;
        this.isRed = isRed;

        drivetrain = new MecanumDrivetrain(op, x, y, theta);
        intake = new Intake(op, isAuto);
        shooter = new Shooter(op, turretTheta);
        wobbleArm = new WobbleArm(op, isAuto);
        logger = new Logger();

        profiler = new ElapsedTime();

        allHubs = op.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        battery = op.hardwareMap.voltageSensor.iterator().next();
        log("Battery Voltage: " + battery.getVoltage() + "v");
        if (battery.getVoltage() < 12.4) {
            startVoltTooLow = true;
        }

        drawField();
        drawRobot(this);
        sendPacket();
    }

    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed) {
        this(op, x, y, theta, PI/2, isAuto, isRed);
    }

    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto) {
        this(op, x, y, theta, isAuto, true);
    }

    // Stop logger and t265
    public void stop() {
        logger.stopLogging();
    }

    // Reset Odometry
    public void resetOdo(double x, double y, double theta) {
        drivetrain.resetOdo(x, y, theta);
    }

    public void update() {
        loopCounter++;
        profiler.reset();
        curTime = System.currentTimeMillis();

        // Powershot Debug
        thetaPositionsR = new double[] {theta0R, theta1R, theta2R};
        thetaPositionsB = new double[] {theta0B, theta1B, theta2B};

        // Track time after start
        if (firstLoop) {
            startTime = curTime;
            lastCycleTime = curTime;
            firstLoop = false;
        }

        profile(1);

        if (!isAuto) {
            if (hgDist >= 100 && turretMode == HIGH_GOAL) {
                setLockMode(MID_GOAL);
            } else if (hgDist < 100 && turretMode == MID_GOAL) {
                setLockMode(HIGH_GOAL);
            }
        }

        // Pre-shoot tasks: Turn on flywheel, move robot to shooting position, mag up, start auto-feed once ready
        if (preShoot) {

            // Set flywheel velocity based on what we want to shoot
            if (highGoal) {
                int v;
                if (turretMode == HIGH_GOAL) {
                    v = calcHGVelocity();
                } else {
                    v = calcMGVelocity();
                }
                shooter.setFlywheelVelocity(v);
                vThresh = v - 70;
            } else {
                shooter.flywheelPS();
                vThresh = Constants.POWERSHOT_VELOCITY - 50;
            }

            // Turn off intake and put mag up
            if (shooter.magHome) {
                intake.off();
                shooter.magShoot();
                log("Mag up");
            }

            // Start auto-feed when mag is up, velocity is high enough, and robot is at position
            if (preShootOverride || (curTime - startShootTime > 1000 && shooter.getFlywheelVelocity() >= vThresh && isAtPoseTurret(shootTargetTheta))) {
                if (highGoal) {
                    shootDelay = highGoalDelay;
                } else {
                    shootDelay = psDelay;
                }
                shoot = true;
                numRings = numRingsPreset;
                shootYOverride = 0;
                shootTime = curTime;
                flickTime = curTime;
                preShoot = false;
                preShootOverride = false;
                log("Ready to shoot " + (highGoal ? "high goal" : "powershot") + ", velocity: " + shooter.getFlywheelVelocity());
                log("Pre shoot time: " +  (curTime - startShootTime) + " ms");
            } else {
                if (shooter.getFlywheelVelocity() < vThresh) {
                    log("Preshoot waiting for shooter v: " + shooter.getFlywheelVelocity() + "/" + vThresh);
                }
                if (!isAtPoseTurret(shootTargetTheta)) {
                    log("Preshoot waiting for turret pos: " + shooter.getTheta() + "/" + shootTargetTheta);
                }
            }

            // If robot does not converge or mag gets stuck
            if (curTime - startShootTime > (isAuto ? 2500 : 4000) && y < 72) {
                if (isAuto || !highGoal) {
                    log("PS: vel: " + (vThresh <= shooter.getFlywheelVelocity())  + ", pose: " + isAtPoseTurret(shootTargetTheta));
                    preShootOverride = true;
                }
                log("Preshoot Timed Out");
            }
        }

        profile(2);

        // Shoot tasks: change/maintain shooting position, auto feed rings
        if (shoot && numRings >= 0 && !shooter.magHome) {
            // Maintain/change robot alignment, set flap
            if (numRings > 0) {
                if (highGoal) {
                    lastTarget = 3;
                } else if (numRings == 3 || curTime - flickTime > flickDelay) {
                    setLockMode(numRings - 1);
                    lastTarget = numRings - 1;
                }
            }

            // Auto feed rings
            if (curTime - shootTime > shootDelay) {
                if (numRings > 0) {
                    // Shoot ring only if robot at position and velocity low enough
                    if (((highGoal && (shootOverride || (shooter.getFlywheelVelocity() >= vThresh && isAtPoseTurret(shootTargetTheta) && y < 72)))
                            || (!highGoal && isAtPoseTurret(shootTargetTheta, PI/100) && turretNotMoving() && y < 72))
                            || (isAuto && curTime - flickTime > flickTimeBackup) || !shooter.feedHome) {

                        log("In shoot Velocity/Target: " + shooter.getFlywheelVelocity() + "/" + shooter.getTargetVelocity());
                        if (!highGoal) {
                            log("PS pos: " + turretGlobalTheta);
                        }

                        if (numRings == 3) {
                            log("Feed ring 1");
                        } else if (numRings == 2) {
                            log("Feed ring 2");
                        } else if (numRings == 1) {
                            log("Feed ring 3");
                        }

                        if (shooter.feedHome) {
                            shooter.feedShoot();
                        } else {
                            shooter.feedHome();
                            shotRings.add(new Ring(x, y, turretGlobalTheta + thetaOffset, vx, vy, w, curTime));
                            numRings--;
                            flickTime = curTime;
                        }
                    } else {
                        if (shooter.getFlywheelVelocity() < vThresh) {
                            log("Shoot waiting for shooter v: " + shooter.getFlywheelVelocity() + "/" + vThresh);
                        }
                        if (!isAtPoseTurret(shootTargetTheta)) {
                            log("Shoot waiting for turret pos: " + shooter.getTheta() + "/" + shootTargetTheta);
                        }
                        if (!highGoal && !turretNotMoving()) {
                            log("Shoot waiting for turret v: " + shooter.getTurretVelocity());
                        }
                    }
                } else {
                    shooter.flywheelOff();
                    shooter.magHome();
                    shoot = false;
                    shootOverride = false;

                    if (!highGoal) {
                        setLockMode(HIGH_GOAL);
                    }

                    log("In shoot Velocity/Target: --------------------");
                    log("Shoot done");
                    log("Total shoot time: " +  (curTime - startShootTime) + " ms");
                    double cycleTime = (curTime - lastCycleTime) / 1000;
                    cycleTotal += cycleTime;
                    cycles++;
                    Log.w("cycle-log", "Cycle " + cycles + ": " + cycleTime + "s");
                    if (cycleTime > longestCycle) {
                        longestCycle = cycleTime;
                    }
                    lastCycleTime = curTime;
                }
                shootTime = curTime;
            }
        }

        shooter.setFlapPosition(flapPos + flapOverride);

        profile(3);

        // Update Position
        drivetrain.updatePose();
        intake.updateBumpers();
        if (isRed) {
            hgDist = Math.sqrt(Math.pow(x - 108, 2) + Math.pow(144 - y, 2));
            mgDist = Math.sqrt(Math.pow(x - 36, 2) + Math.pow(144 - y, 2));
        } else {
            hgDist = Math.sqrt(Math.pow(x - 36, 2) + Math.pow(144 - y, 2));
            mgDist = Math.sqrt(Math.pow(x - 108, 2) + Math.pow(144 - y, 2));
        }

        turretGlobalTheta = shooter.getTheta() + theta - PI/2;

        profile(4);

        if (turretMode != NONE) {
            if (turretReset) {
                shooter.setTurretPower(0.1);
            } else {
                shooter.updateTurret(theta, drivetrain.commandedW);
            }
            updateTurret();
        }

        profile(5);

        // Calculate Motion Info
        double timeDiff = curTime / 1000 - prevTime;
        x = drivetrain.x;
        y = drivetrain.y;
        theta = drivetrain.theta;
        vx = (x - prevX) / timeDiff; vy = (y - prevY) / timeDiff; w = (theta - prevTheta) / timeDiff;
        ax = (vx - prevVx) / timeDiff; ay = (vy - prevVy) / timeDiff; a = (w - prevW) / timeDiff;

        // Remember Previous Motion Info
        prevX = x; prevY = y;
        prevTheta = theta;
        prevTime = curTime / 1000;
        prevVx = vx; prevVy = vy; prevW = w;

        profile(6);

        // Log Data
        if (loopCounter % loggerUpdatePeriod == 0) {
            logger.logData(curTime - startTime, x, y, theta, turretGlobalTheta, vx, vy, w, ax, ay, a, numRings, shooter.magHome, shooter.feedHome, lastTarget, cycles, cycleTotal / cycles);
        }

        profile(7);

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("0", "Starting Battery Voltage < 12.4!!!!");
        }
        addPacket("1 X", round(x));
        addPacket("2 Y", round(y));
        addPacket("3 Theta", round(theta));
        addPacket("4 Turret Theta", round(turretGlobalTheta) + " " + round(shootTargetTheta) + " " + round(shooter.getTheta()) + " " + shooter.getTurretVelocity());
        addPacket("5 Shooter Velocity", shooter.getFlywheelVelocity() + " " + vThresh);
        addPacket("6 numRings", numRings);
        addPacket("7 shoot", preShoot  + " " + shoot + " " + highGoal + " " + turretMode.name() + " " + round(shootTargetTheta));
        addPacket("8 Run Time", (curTime - startTime) / 1000);
        addPacket("9 Update Frequency (Hz)", round(1 / timeDiff));
        addPacket("Pod Zeroes", drivetrain.zero1 + ", " + drivetrain.zero2 + ", " + drivetrain.zero3);
        addPacket("offsets", round(thetaOffset) + " " + round(flapOverride) + " " + shootYOffset);
        addPacket("ms", round(timeDiff * 1000));
        if (!isAuto) {
            addPacket("Cycle Time", (curTime - lastCycleTime) / 1000);
            addPacket("Average Cycle Time", round(cycleTotal / cycles));
            addPacket("Cycles", cycles);
        }

        // Dashboard Drawings
        drawField();
        drawRobot(this);
        for (Ring ring : ringPos) {
            if (ring.getX() != x && ring.getY() != y) {
                drawRing(ring);
            }
        }
        int shotRingCount = 0;
        while (shotRingCount < shotRings.size()) {
            Ring ring = shotRings.get(shotRingCount);
            ring.updatePose(curTime);
            double[] ringCoords = ring.getAbsCoords();
            if (0 < ringCoords[0] && ringCoords[0] < 144 && 0 < ringCoords[1] && ringCoords[1] < 144) {
                shotRingCount++;
                drawRing(ring);
            } else {
                shotRings.remove(shotRingCount);
            }
        }
        sendPacket();

        profile(8);

        // Clear bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        profile(9);
    }

    public void highGoalShoot() {
        highGoalShoot(3, true);
    }

    public void highGoalShoot(boolean useFlap) {
        highGoalShoot(3, useFlap);
    }

    public void highGoalShoot(int numRings) {
        highGoalShoot(numRings, true);
    }

    // Set variables for high goal shoot
    public void highGoalShoot(int numRings, boolean useFlap) {
        if (!preShoot && !shoot) {
            preShoot = true;
            highGoal = true;
            numRingsPreset = numRings;
            if (numRings != 3) {
                log("Shooting with " + numRings + " rings");
            }
            startShootTime = curTime;
            if (useFlap) {
                flapPos = Constants.FLAP_UP_POS;
            } else {
                flapPos = Constants.FLAP_DOWN_POS;
            }

            if (isAuto) {
                drivetrain.stop();
            }

            shootTargetTheta = calculateShootTheta();
            log("High goal shoot initiated");
        }
    }

    // Set variables for powershot shoot
    public void powerShotShoot() {
        if (!preShoot && !shoot) {
            preShoot = true;
            highGoal = false;
            numRingsPreset = 3;
            startShootTime = curTime;
            flapPos = Constants.FLAP_DOWN_POS;
            drivetrain.stop();
            setLockMode(PS_R);
            log("Powershot shoot initiated");
        }
    }

    // Cancel shoot sequence
    public void cancelShoot() {
        preShoot = false;
        shoot = false;
        numRings = 0;
        shootYOverride = 0;
        shooter.flywheelOff();
        shooter.magHome();
        shooter.feedHome();
        log("Shoot cancelled");
    }

    public void setLockMode(TurretMode lockMode) {
        setLockMode(lockMode.ordinal());
    }

    public void setLockMode(int lockMode) {
        if (lockMode == 0) {
            turretMode = PS_L;
        } else if (lockMode == 1) {
            turretMode = PS_C;
        } else if (lockMode == 2) {
            turretMode = PS_R;
        } else if (lockMode == 3) {
            turretMode = HIGH_GOAL;
            if (isRed) {
                setLock(shootXCorR[lockMode], shootYCor);
            } else {
                setLock(shootXCorB[lockMode], shootYCor);
            }
        } else if (lockMode == 4) {
            turretMode = MID_GOAL;
            if (isRed) {
                setLock(36, shootYCor);
            } else {
                setLock(108, shootYCor);
            }
        } else {
            turretMode = NONE;
        }
    }

    public void setLock(double x, double y) {
        lockX = x;
        lockY = y;
    }

    public void updateTurret() {
        /* power1- (76.5,144,24)
           power2- (84,144,24)
           power3- (91.5,144,24)
           high goal- (108,144,35.5) */

        if (calculateShootTheta() == Double.MAX_VALUE) {
            shooter.setTargetTheta(Double.MAX_VALUE);
        } else {
            if (turretMode == HIGH_GOAL || turretMode == MID_GOAL) {
                shooter.setTargetTheta(calculateShootTheta() - thetaOffset);
                shootTargetTheta = calculateShootTheta() - thetaOffset;
            } else if (turretMode != NONE) {
                if (isRed) {
                    shooter.setTargetTheta(thetaPositionsR[turretMode.ordinal()]);
                    shootTargetTheta = thetaPositionsR[turretMode.ordinal()];
                } else {
                    shooter.setTargetTheta(thetaPositionsB[turretMode.ordinal()]);
                    shootTargetTheta = thetaPositionsB[turretMode.ordinal()];
                }
            }
        }
    }

    public double calculateShootTheta() {
        if (y > 96) {
            return Double.MAX_VALUE;
        }

        double shooterX = x + (turretMode != NONE && 0.5 < abs(vx) ? vx : 0) * Shooter.RING_FLIGHT_TIME + Shooter.TURRET_DX * sin(theta) + Shooter.TURRET_DY * cos(theta);
        double shooterY = y + (turretMode != NONE && 0.5 < abs(vy) ? vy : 0) * Shooter.RING_FLIGHT_TIME - Shooter.TURRET_DX * cos(theta) + Shooter.TURRET_DY * sin(theta);
        double dx = lockX - shooterX;
        double dy = lockY - shooterY;

        drawLine(shooterX, shooterY, lockX, lockY, "blue");
        return Math.atan2(dy, dx);
    }

    public int calcHGVelocity() {
        return (int) (1450 + 2 * hgDist);
    }

    public int calcMGVelocity() {
        return (int) (1007 + 4.78 * mgDist);
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
        if (abs(theta - thetaTarget) > PI) {
            thetaControl = abs(theta - thetaTarget) / (theta - thetaTarget) * (abs(theta - thetaTarget) - 2*PI);
        } else {
            thetaControl = theta - thetaTarget;
        }

        drawDrivetrain(xTarget, yTarget, thetaTarget, "blue");

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
    public boolean isAtPose(double targetX, double targetY) {
        return isAtPose(targetX, targetY, 0, xyTolerance, xyTolerance, 2*PI);
    }

    public boolean isAtPose(double targetX, double targetY, double targetTheta) {
        return isAtPose(targetX, targetY, targetTheta, xyTolerance, xyTolerance, thetaTolerance);
    }

    // Check if robot is at a certain point/angle (custom tolerance)
    public boolean isAtPose(double targetX, double targetY, double xTolerance, double yTolerance) {
        return abs(x - targetX) < xTolerance && abs(y - targetY) < yTolerance;
    }
    public boolean isAtPose(double targetX, double targetY, double targetTheta, double xTolerance, double yTolerance, double thetaTolerance) {
        return abs(x - targetX) < xTolerance && abs(y - targetY) < yTolerance && abs(theta - targetTheta) < thetaTolerance;
    }

    // Check if robot is at a certain point, turret at certain angle (default tolerance)
    public boolean isAtPoseTurret(double targetX, double targetY, double turretTheta) {
        return isAtPose(targetX, targetY, turretTheta, xyTolerance, xyTolerance, turretTolerance);
    }

    // Check if robot is at a certain point, turret at certain angle (custom tolerance)
    public boolean isAtPoseTurret(double targetX, double targetY, double turretTheta, double xTolerance, double yTolerance, double turretTolerance) {
        return abs(x - targetX) < xTolerance && abs(y - targetY) < yTolerance && abs(turretGlobalTheta - turretTheta) < turretTolerance;
    }

    public boolean isAtPoseTurret(double turretTheta, double turretTolerance) {
        return abs(turretGlobalTheta - turretTheta) < turretTolerance;
    }

    public boolean isAtPoseTurret(double turretTheta) {
        return abs(turretGlobalTheta - turretTheta) < turretTolerance;
    }

    public boolean notMoving() {
        if (isAuto || !highGoal) {
            return notMoving(3.0, 0.2);
        }
        return true;
    }

    public boolean notMoving(double xyThreshold, double thetaThreshold) {
        return (Math.hypot(vx, vy) < xyThreshold && abs(w) < thetaThreshold);
    }

    public boolean turretNotMoving() {
        return shooter.getTurretVelocity() < 0.1;
    }

    // Logging
    public static void log(String message) {
        Log.w("robot-log", message);
    }

    private void profile(int num) {
        Log.w("profiler", num + ": " + profiler.milliseconds());
    }

    @SuppressLint("DefaultLocale")
    public static double round(double num) {
        return Double.parseDouble(String.format("%.5f", num));
    }
}
