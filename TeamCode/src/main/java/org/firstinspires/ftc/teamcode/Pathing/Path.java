package org.firstinspires.ftc.teamcode.Pathing;

import java.util.ArrayList;

public class Path {
    private ArrayList<Spline[]> splines = new ArrayList<>();
    private ArrayList<Double> waypointTimes = new ArrayList<>();
    private ArrayList<Waypoint> waypoints;
    private double totaltime;

    public Path(ArrayList<Waypoint> waypoints) {
        // Defining waypoint Arraylist
        this.waypoints = waypoints;

        // Define Splinegenerater, All Splines Necessary
        SplineGenerator splinegen = new SplineGenerator();

        // Find Total Time to Make Sure Nothing is Going Wrong on Pose Calls
        totaltime = waypoints.get(waypoints.size() - 1).time;

        for (int i = 0; i < waypoints.size() - 1; i++) {
            // Get Relevant waypoints
            Waypoint waypoint1 = waypoints.get(i);
            Waypoint waypoint2 = waypoints.get(i + 1);

            // Define All Variables For Spline
            double startx = waypoint1.x;
            double starty = waypoint1.y;
            double endx = waypoint2.x;
            double endy = waypoint2.y;
            double starttheta = waypoint1.theta;
            double endtheta = waypoint2.theta;
            double startvx = waypoint1.vx;
            double endvx = waypoint2.vx;
            double startvy = waypoint1.vy;
            double endvy = waypoint2.vy;
            double startax = waypoint1.ax;
            double endax = waypoint2.ax;
            double startay = waypoint1.ay;
            double enday = waypoint2.ay;
            double time = waypoint2.time - waypoint1.time;

            // Make Sure waypoints Are Correct
            assert waypoint2.time > waypoint1.time: "Waypoint times are not correct";

            // Generate Splines and Add Them to Array
            Spline[] segment = splinegen.SplineBetweenTwoPoints(startx, starty, endx, endy, starttheta, endtheta, startvx, endvx, startvy, endvy, startax, endax, startay, enday, time);
            splines.add(segment);

            // Add Time
            waypointTimes.add(waypoint2.time);
        }
    }

    public Pose getRobotPose(double time) {
        int splineindex = 0;

        if (totaltime <= time) {
            splineindex = waypointTimes.size() - 1;
        } else {
            for (int i = 0; i < waypointTimes.size(); i++) {
                if (waypointTimes.get(i) > time) {
                    splineindex = i;
                    break;
                }
            }
        }
        double splinetime = time - waypoints.get(splineindex).time;
        Spline[] currentspline = splines.get(splineindex);
        double x = currentspline[0].position(splinetime);
        double y = currentspline[1].position(splinetime);
        double vx = currentspline[0].velocity(splinetime);
        double vy = currentspline[1].velocity(splinetime);
        double theta = Math.atan2(vy, vx);
        double w = (vx * currentspline[1].accel(splinetime) - vy * currentspline[0].accel(splinetime)) / (Math.pow(vx, 2) + Math.pow(vy, 2));
        return new Pose(x, y, theta, vx, vy, w);
    }
}
