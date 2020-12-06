package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;

@SuppressLint("SdCardPath")
public class Logger {

    private static final String basePath = "/sdcard/FIRST/robotLogs/RobotData";
    private static FileWriter fileWriter;
    private static BufferedReader bufferedReader;
    private String data = "";

    /**
     * Creates a new file and writes first row
     * This method must be used before logging
     */
    public void startLogging() {
        try {
            data = "";
            File robotDataLog = new File(getLogName(true));
            fileWriter = new FileWriter(robotDataLog);
            fileWriter.write("Timestamp,SinceStart,X,Y,Theta,VelocityX,VelocityY,VelocityTheta,AccelX,AccelY,AccelTheta,NumRings,MagHome,FeedHome\n");
        } catch (Exception e) {e.printStackTrace();}
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
        Robot.log(basePath + (getLastFileNumber() + 1) + ".csv");
        if (fileWrite) {
            return basePath + (getLastFileNumber() + 1) + ".csv";
        } else {
            return basePath + getLastFileNumber() + ".csv";
        }
    }

    /**
     * Adds data to string
     */
    public void logData(double timeSinceSt, double x, double y, double theta, double velocityX, double velocityY, double velocityTheta,
                        double accelX, double accelY, double accelTheta, int numRings, boolean magHome, boolean feedHome) {
        @SuppressLint("SimpleDateFormat") SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss.SSS"); Date d = new Date();
        data += df.format(d)+","+timeSinceSt+","+x+","+y+","+theta+","+velocityX+","+velocityY+","+velocityTheta+","+
                accelX+","+accelY+","+accelTheta+","+numRings+","+magHome+","+feedHome+"\n";
    }

    /**
     * Writes string to file, closes file
     */
    public void stopLogging() {
        try {
            fileWriter.write(data);
            fileWriter.close();
        }
        catch (Exception e) {e.printStackTrace();}
    }

    /**
     * Reads position from last written file
     */
    public static double[] readPos() {
        String curLine; int lineNum = 0;
        double[] robotPos = new double[3];

        try {
            bufferedReader = new BufferedReader(new FileReader(getLogName(false)));
            while ((curLine = bufferedReader.readLine()) != null) {
                if (lineNum != 0) {
                    String[] data = curLine.split(","); //Robot.log(Arrays.toString(data));
                    robotPos[0] = Double.parseDouble(data[2]);
                    robotPos[1] = Double.parseDouble(data[3]);
                    robotPos[2] = Double.parseDouble(data[4]);
                    //Robot.log(Arrays.toString(robotPos));
                }
                lineNum++;
            }
            bufferedReader.close();
            Robot.log("Starting At " + Arrays.toString(robotPos));
        } catch (Exception ex) {
            robotPos[0] = 0; robotPos[1] = 0; robotPos[2] = 0;
            Robot.log("read error, using default values :-(");
            ex.printStackTrace();
        }

        return robotPos;
    }
}