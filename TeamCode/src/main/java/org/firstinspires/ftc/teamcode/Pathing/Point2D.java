package org.firstinspires.ftc.teamcode.Pathing;

public class Point2D {

    private double x;
    private double y;

    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double distance(Point2D p) {
        return Math.sqrt(Math.pow((p.getX() - x), 2) + Math.pow((p.getY() - y), 2));
    }
}
