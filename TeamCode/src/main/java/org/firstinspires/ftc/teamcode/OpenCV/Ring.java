package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.RobotClasses.Shooter;

import java.util.ArrayList;
import java.util.Comparator;

import static org.firstinspires.ftc.teamcode.OpenCV.RingLocator.RingLocator.*;

public class Ring {
    private double relX;
    private double relY;
    private double absX;
    private double absY;
    private double vx;
    private double vy;
    private double startX;
    private double startY;
    private double startTime;

    private static double DIST_THRESH = 1.5;

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

    public Ring(double startX, double startY, double theta, double vx, double vy, double omega, double startTime) {
        this.startX = startX + Shooter.SHOOTER_DX * Math.sin(theta);
        this.startY = startY - Shooter.SHOOTER_DX * Math.cos(theta);
        this.absX = this.startX;
        this.absY = this.startY;
        this.vx = vx - Shooter.SHOOTER_DX * omega * Math.sin(theta) + Shooter.RING_SPEED * Math.cos(theta + Shooter.INITIAL_ANGLE);
        this.vy = vy + Shooter.SHOOTER_DX * omega * Math.cos(theta) + Shooter.RING_SPEED * Math.sin(theta + Shooter.INITIAL_ANGLE);
        this.startTime = startTime;
    }

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

        // Remove rings that are too close to each other
        i = 0;
        while (i < rings.size()) {
            double xi = rings.get(i).getX();
            double yi = rings.get(i).getY();
            int j = i + 1;

            while (j < rings.size()) {
                Ring ring = rings.get(j);
                if (Math.abs(xi - ring.getX()) < DIST_THRESH && Math.abs(yi - ring.getY()) < DIST_THRESH) {
                    rings.remove(j);
                } else {
                    j++;
                }
            }
            i++;
        }

        // Sort rings based on y coordinate
        rings.sort(Comparator.comparingDouble(r -> r.getY()));

        // Return up to three rings
        if (rings.size() > 3) {
            rings = new ArrayList<>(rings.subList(0, 3));
        }

        // Determine left or right sweep
        if (rings.size() > 0) {
            if (rings.get(rings.size() - 1).getY() - rings.get(0).getY() > 8) {
                Ring closest = rings.remove(0);
                if (closest.getX() <= robotX + 9) {
                    rings.sort(Comparator.comparingDouble(r -> r.getX()));
                } else {
                    rings.sort(Comparator.comparingDouble(r -> -r.getX()));
                }
                rings.add(0, closest);
            } else {
                rings.sort(Comparator.comparingDouble(r -> r.getX()));
            }
        }

        return rings;
    }

    // Return a sorted list with up to three coordinate-filtered rings
    public static ArrayList<Ring> getRingCoords(ArrayList<Ring> rings, double robotX, double robotY) {
        return getRingCoords(rings, minX, minY, maxX, maxY, robotX, robotY);
    }

    // Calculate ring absolute coordinates using relative coordinates and robot position
    public void calcAbsCoords(double robotX, double robotY, double robotTheta) {
        absX = robotX + relX * Math.sin(robotTheta) + relY * Math.cos(robotTheta);
        absY = robotY - relX * Math.cos(robotTheta) + relY * Math.sin(robotTheta);
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        if (startTime == 0) {
            return "R(" + String.format("%.3f", relX) + ", " + String.format("%.3f", relY) + "), A(" + String.format("%.3f", absX) + ", " + String.format("%.3f", absY) + ")";
        } else {
            return "A(" + String.format("%.3f", absX) + ", " + String.format("%.3f", absY) + "), V(" + String.format("%.3f", vx) + ", " + String.format("%.3f", vy) + "), S(" + String.format("%.3f", startTime) + ")";
        }
    }

    public Ring clone() {
        return new Ring(relX, relY, absX, absY);
    }

    public double[] driveToRing(double robotX, double robotY) {
        return new double[] {absX, absY, Math.atan2(absY - robotY, absX - robotX)};
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

    public void updatePose(double curTime) {
        double dt = (curTime - startTime) / 1000;
        absX = startX + vx * dt;
        absY = startY + vy * dt;
    }
}