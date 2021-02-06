package org.firstinspires.ftc.teamcode.OpenCV;

import java.util.ArrayList;

public class Ring {
    private double relX;
    private double relY;
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

    public Ring(double relX, double relY, double robotX, double robotY, double robotTheta) {
        this.relX = relX;
        this.relY = relY;
        calcAbsCoords(robotX, robotY, robotTheta);
    }

    // Return a sorted list with up to three coordinate-filtered rings
    public static ArrayList<Ring> getRingCoords(ArrayList<Ring> rings, double minX, double minY, double maxX, double maxY) {
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

        // Sort rings based on ascending x value
        rings.sort((r1, r2) -> Double.compare(r1.getX(), r2.getX()));

        // Return up to three rings
        if (rings.size() > 3) {
            return new ArrayList<Ring>(rings.subList(0, 3));
        } else {
            return rings;
        }
    }

    // Calculate ring absolute coordinates using relative coordinates and robot position
    public void calcAbsCoords(double robotX, double robotY, double robotTheta) {
        absX = robotX + relX * Math.sin(robotTheta) + relY * Math.cos(robotTheta);
        absY = robotY + relX * Math.cos(robotTheta) + relY * Math.sin(robotTheta);
    }

    public double[] getRelCoords() {
        return new double[] {relX, relY};
    }

    public double[] getAbsCoords() {
        return new double[] {absX, absY};
    }

    public void setRelCoords(double[] relCoords) {
        relX = relCoords[0];
        relY = relCoords[1];
    }

    public void setAbsCoords(double[] absCoords) {
        absX = absCoords[0];
        absY = absCoords[1];
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

    public void setRelX(double relX) {
        this.relX = relX;
    }

    public void setRelY(double relY) {
        this.relY = relY;
    }

    public void setAbsX(double absX) {
        this.absX = absX;
    }

    public void setAbsY(double absY) {
        this.absY = absY;
    }
}