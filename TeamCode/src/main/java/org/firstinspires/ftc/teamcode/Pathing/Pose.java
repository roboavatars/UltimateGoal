package org.firstinspires.ftc.teamcode.Pathing;

public class Pose {
    private double x;
    private double y;
    private double theta;
    private double vx;
    private double vy;
    private double w;

    public Pose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose(double x, double y, double theta, double vx, double vy, double w) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.vx = vx;
        this.vy = vy;
        this.w = w;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }

    public double getW() {
        return w;
    }
}
