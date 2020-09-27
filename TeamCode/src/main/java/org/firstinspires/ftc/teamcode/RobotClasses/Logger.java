package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

@SuppressLint("SdCardPath")
public class Logger {

    private static String basePath = "/sdcard/FIRST/robotLogs/RobotData";
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
            fileWriter.write("Timestamp,SinceStart,X,Y,Theta,VelocityX,VelocityY,VelocityTheta,AccelX,AccelY,AccelTheta\n");
        } catch (Exception e) {e.printStackTrace();}
    }

    /**
     * Searches the list of files to find the last file number that exists
     */
    public static double getLastFileNumber() {
        int logNum = 1;
        while (true) {
            File currentFile = new File(basePath + logNum + ".csv");
            if (!currentFile.exists()) break;
            logNum++;
        }
        return logNum - 1;
    }

    /**
     * Takes the file number from getLastFileNumber() and converts it to a file name
     */
    private static String getLogName(boolean fileWrite) {
        System.out.println(basePath + (int)(getLastFileNumber()+ 1) + ".csv");
        if (fileWrite) return basePath + (int)(getLastFileNumber()+ 1) + ".csv";
        else return basePath + getLastFileNumber() + ".csv";
    }

    /**
     * Adds data to string
     */
    public void logData(double timeSinceSt, double x, double y, double theta, double velocityx, double velocityy, double velocitytheta, double accelx, double accely, double accelTheta) {
        @SuppressLint("SimpleDateFormat") SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss.SSS"); Date d = new Date();
        data += df.format(d)+","+timeSinceSt+","+x+","+y+","+theta+","+velocityx+","+velocityy+","+velocitytheta+","+accelx+","+accely+","+accelTheta+"\n";
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
                    String[] data = curLine.split(","); //System.out.println(Arrays.toString(data));
                    robotPos[0] = Double.parseDouble(data[2]);
                    robotPos[1] = Double.parseDouble(data[3]);
                    robotPos[2] = Double.parseDouble(data[4]);
                    //System.out.println(Arrays.toString(robotPos);
                }
                lineNum++;
            }
            bufferedReader.close();
        } catch (Exception ex) {
            robotPos[0] = 0; robotPos[1] = 0; robotPos[2] = 0;
            System.out.println("read error, using default values :-(");
            ex.printStackTrace();
        }

        return robotPos;
    }

    public ArrayList<double[]> replay(String path) {
        String curLine;
        int lineNum = 0;
        ArrayList<double[]> dataArray = new ArrayList<>();
        try {
            File robotDataLog = new File(path);
            bufferedReader = new BufferedReader(new FileReader(robotDataLog));

            while ((curLine = bufferedReader.readLine()) != null) {
                if (lineNum != 0) {
                    String[] data = curLine.split(",");
                    dataArray.add(new double[]{Double.parseDouble(data[1]), Double.parseDouble(data[2]), Double.parseDouble(data[3]), Double.parseDouble(data[4])});
                }
                lineNum++;
            }
            bufferedReader.close();
        } catch (IOException e) {e.printStackTrace();}
        return dataArray;
    }
}