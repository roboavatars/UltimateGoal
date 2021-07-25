package org.firstinspires.ftc.teamcode.Debug;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.stream.Collectors;

@SuppressLint("SdCardPath")
public class Logger extends Thread {

    private static final String basePath = "/sdcard/FIRST/robotLogs/RobotData";
    private static FileWriter fileWriter;
    private int logCounter;
    private int writeCounter;

    private SimpleDateFormat df;
    private double timeSinceSt;
    private double x, y, theta;
    private double turretTheta;
    private double vx, vy, w;
    private double ax, ay, alpha;
    private int numRings;
    private boolean magHome, feedHome;
    private int lastTarget;
    private int numCycles;
    private double avgCycleTime;

    /**
     * Creates a new file and writes initial rows
     */
    public void startLogging(boolean isAuto, boolean isRed) {
        try {
            File robotDataLog = new File(getLogName(true));
            fileWriter = new FileWriter(robotDataLog);
            fileWriter.write("# " + (isAuto ? "Auto" : "Teleop") + "\n" + (isRed ? "Red" : "Blue") + "\n");
            fileWriter.write("Timestamp,SinceStart,X,Y,Theta,TurretTheta,VelocityX,VelocityY,VelocityTheta,AccelX,AccelY,AccelTheta,NumRings,MagHome,FeedHome,LastTarget,Cycles,AvgCycle\n");
            logCounter = 0;
            writeCounter = 0;
            start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Searches the list of files to find the last file number that exists
     */
    public static int getLastFileNumber() {
        int logNum = 1;
        while (true) {
            File currentFile = new File(basePath + logNum + ".csv");
            if (!currentFile.exists()) {
                break;
            }
            logNum++;
        }
        return logNum - 1;
    }

    /**
     * Takes the file number from getLastFileNumber() and converts it to a file name
     */
    private static String getLogName(boolean fileWrite) {
        if (fileWrite) {
            Robot.log("Writing to: " + basePath + (getLastFileNumber() + 1) + ".csv");
            return basePath + (getLastFileNumber() + 1) + ".csv";
        } else {
            Robot.log("Reading from: " + basePath + getLastFileNumber() + ".csv");
            return basePath + getLastFileNumber() + ".csv";
        }
    }

    /**
     * Writes log data in separate thread
     */
    @Override
    public void run() {
        while (!isInterrupted()) {
            if (writeCounter < logCounter) {
                try {
                    fileWriter.write(df.format(new Date()) + "," + timeSinceSt + "," + x + "," + y + "," + theta + "," + turretTheta + "," + vx + "," + vy + "," + w + "," + ax + "," + ay + "," + alpha + "," + numRings + "," + magHome + "," + feedHome + "," + lastTarget + "," + numCycles + "," + avgCycleTime + "\n");
                    writeCounter++;
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

    /**
     * Saves log data
     */
    @SuppressLint("SimpleDateFormat")
    public void logData(double timeSinceSt, double x, double y, double theta, double turretTheta, double vx, double vy, double w, double ax, double ay, double alpha, int numRings, boolean magHome, boolean feedHome, int lastTarget, int numCycles, double avgCycleTime) {
        df = new SimpleDateFormat("HH:mm:ss.SSS");
        this.timeSinceSt = timeSinceSt;
        this.x = x; this.y = y; this.theta = theta;
        this.turretTheta = turretTheta;
        this.vx = vx; this.vy = vy; this.w = w;
        this.ax = ax; this.ay = ay; this.alpha = alpha;
        this.numRings = numRings;
        this.magHome = magHome; this.feedHome = feedHome;
        this.lastTarget = lastTarget;
        this.numCycles = numCycles;
        this.avgCycleTime = avgCycleTime;

        logCounter++;
    }

    /**
     * Closes file
     */
    public void stopLogging() {
        interrupt();
        try {
            fileWriter.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Reads position from last written file
     */
    public static double[] readPos() {
        double[] robotPos = new double[] {0, 0, 0, 0};

        try {
            BufferedReader bufferedReader = new BufferedReader(new FileReader(getLogName(false)));
            List<String> lines = bufferedReader.lines().collect(Collectors.toList());
            int isRed = lines.get(1).contains("Red") ? 1 : 0;
            String[] data = lines.get(lines.size() - 2).split(",");
            robotPos = new double[] {isRed, Double.parseDouble(data[2]), Double.parseDouble(data[3]), Double.parseDouble(data[4]), Double.parseDouble(data[5])};

            bufferedReader.close();
            Robot.log("Starting At " + Arrays.toString(robotPos));
        } catch (Exception e) {
            Robot.log("read error, using default values :-(");
            e.printStackTrace();
        }
        return robotPos;
    }
}