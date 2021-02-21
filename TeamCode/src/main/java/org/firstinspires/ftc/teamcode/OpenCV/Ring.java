package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.OpenCV.RingLocator.RingLocatorPipeline;

import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.OpenCV.RingLocator.RingLocator.*;

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

    // Return a sorted list with up to two coordinate-filtered rings
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
        // Sort rings based on distance
        rings.sort((r1, r2) -> Double.compare(r1.getAbsDist(robotX, robotY), r2.getAbsDist(robotX, robotY)));

        // Return up to two rings
        if (rings.size() > 2) {
            rings = new ArrayList<>(rings.subList(0, 2));
        }

//        if (rings.size() == 3) {
//            Ring closest = rings.get(0);
//            // Find closet ring after first ring
//            if (rings.get(1).getAbsDist(closest.absX, closest.absY) > rings.get(2).getAbsDist(closest.absX, closest.absY)) {
//                Collections.swap(rings, 1, 2);
//            }
//        }

        return rings;
    }

    public static ArrayList<Ring> getRingCoords(ArrayList<Ring> rings, double robotX, double robotY) {
        return getRingCoords(rings, minX, minY, maxX, maxY, robotX, robotY);
    }

    // Calculate ring absolute coordinates using relative coordinates and robot position
    public void calcAbsCoords(double robotX, double robotY, double robotTheta) {
        absX = robotX + relX * Math.sin(robotTheta) + relY * Math.cos(robotTheta);
        absY = robotY + RingLocatorPipeline.CAM_FRONT - relX * Math.cos(robotTheta) + relY * Math.sin(robotTheta);
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return "R(" + String.format("%.3f", relX) + ", " + String.format("%.3f", relY) + "), A(" + String.format("%.3f", absX) + ", " + String.format("%.3f", absY) + ")";
    }

    public Ring clone() {
        return new Ring(relX, relY, absX, absY);
    }

    public double[] driveToRing(double robotX, double robotY) {
        return new double[] {absX, absY, Math.atan2(absY - robotY, absX - robotX)};
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