package org.firstinspires.ftc.teamcode.Pathing;

public class SplineGenerator {
    public Spline[] SplineBetweenTwoPoints(double startx, double starty, double endx, double endy,
                                           double starttheta, double endtheta, double startxdot,
                                           double endxdot, double startydot, double endydot,
                                           double startxdotdot, double endxdotdot, double startydotdot,
                                           double endydotdot, double time) {
        Spline xspline = new Spline(startx, startxdot, startxdotdot, endx, endxdot, endxdotdot, time);
        Spline yspline = new Spline(starty, startydot, startydotdot, endy, endydot, endydotdot, time);
        Spline[] splines = {xspline, yspline};

        return splines;
    }
}
