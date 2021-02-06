package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Log;
import java.util.ArrayList;
import java.util.Arrays;

public class Ring {
    private final double relX;
    private final double relY;
    private double absX;
    private double absY;

    public Ring(double relX, double relY, double absX, double absY) {
        this.relX = relX;
        this.relY = relY;
        this.absX = absX;
        this.absY = absY;
    }

    public Ring(double relX, double relY) {
        this.relX = relX;
        this.relY = relY;
    }

//    public Ring(double absX, double absY, boolean isAbs) {
//        this.absX = absX;
//        this.absY = absY;
//    }
//
//    public Ring(double relX, double relY, double robotX, double robotY, double robotTheta) {
//        this.relX = relX;
//        this.relY = relY;
//        calcAbsCoords(robotX, robotY, robotTheta);
//    }

    // Return a sorted list with up to three coordinate-filtered rings
    public static ArrayList<Ring> getRingCoords(ArrayList<Ring> rings, double minX, double minY, double maxX, double maxY, double robotX, double robotY) {
        // Remove rings out of bounds
        int i = 0;
        while (i < rings.size()) {
            Ring ring = rings.get(i);
            if (ring.getX() < minX || ring.getX() > maxX || ring.getY() < minY || ring.getY() > maxY) {
                rings.remove(i);
            } else {
                i++;
            }
        }

        Log.w("robot-log", "rings2: " + rings);

        // Sort rings based on ascending x value
        rings.sort((r1, r2) -> Double.compare(r1.getX(), r2.getX()));

        Log.w("robot-log", "rings3: " + rings);

        // Return up to three rings
        if (rings.size() > 3) {
            return new ArrayList<Ring>(rings.subList(0, 3));
        } else if (rings.size() == 0) {
            return new ArrayList<Ring>(Arrays.asList(new Ring(0, 0, robotX, robotY)));
        } else {
            return rings;
        }
    }

    // Calculate ring absolute coordinates using relative coordinates and robot position
    public void calcAbsCoords(double robotX, double robotY, double robotTheta) {
        absX = robotX + relX * Math.sin(robotTheta) + relY * Math.cos(robotTheta);
        absY = robotY - relX * Math.cos(robotTheta) + relY * Math.sin(robotTheta);
    }

    public String toString() {
        return "Relative: (" + relX + ", " + relY + "), Absolute: (" + absX + ", " + absY + ")";
    }

    public double[] getRelCoords() {
        return new double[] {relX, relY};
    }

    public double[] getAbsCoords() {
        return new double[] {absX, absY};
    }

    public double getRelDist() {
        return Math.hypot(relX, relY);
    }

    public double getAbsDist(double robotX, double robotY) {
        return Math.hypot(absX - robotX, absY - robotY);
    }

    public double getRelX() {
        return relX;
    }

    public double getRelY() {
        return relY;
    }

    public double getX() {
        return absX;
    }

    public double getY() {
        return absY;
    }
}