package org.firstinspires.ftc.teamcode.Splines;

public class Waypoint {
    public double x;
    public double y;
    public double theta;
    public double xdot;
    public double ydot;
    public double xdotdot;
    public double ydotdot;
    public double time;

    /**
     * Generates waypoint for differential drive constraints: velocity and acceleration are in
     * the same direction as heading
     */
    public Waypoint(double x, double y, double theta, double velocity, double acceleration, double angularVelocity, double time){
        assert velocity==0: "Waypoint Velocity is zero";
        this.x = x;
        this.y = y;
        this.theta = theta;
        xdot = velocity * Math.cos(theta);
        ydot = velocity * Math.sin(theta);
        xdotdot = acceleration * Math.cos(theta) - velocity * Math.sin(theta) * angularVelocity;
        ydotdot = acceleration * Math.sin(theta) + velocity * Math.cos(theta) * angularVelocity;
        this.time = time;

    }

    /**
     * Generates Waypoint for Mecanum drive constraints: velocity and acceleration vectors can
     * be in any direction.
     */
    public Waypoint(double x, double y, double theta, double xdot, double ydot, double xdotdot, double ydotdot, double time){

        assert (Math.sqrt(Math.pow(xdot,2) + Math.pow(ydot,2)) != 0) : "Waypoint Velocity is zero";
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.xdot = xdot;
        this.ydot = ydot;
        this.xdotdot = xdotdot;
        this.ydotdot = ydotdot;
        this.time = time;
    }
}
