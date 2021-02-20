package org.firstinspires.ftc.teamcode.Pathing;

public class Pose {
    public final double x;
    public final double y;
    public final double theta;
    public final double vx;
    public final double vy;
    public final double w;

    public Pose(double x, double y, double theta) {
        this(x, y, theta, 0, 0, 0);
    }

    public Pose(double x, double y, double theta, double vx, double vy, double w) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.vx = vx;
        this.vy = vy;
        this.w = w;
    }
}
