package org.firstinspires.ftc.teamcode.Splines;

import java.util.ArrayList;

public class Path {
    private ArrayList<Spline[]> splines = new ArrayList<Spline[]>();
    private ArrayList<Double> waypointTimes = new ArrayList<Double>();
    private ArrayList<Waypoint> waypoints;
    private double totaltime;

    public Path(ArrayList<Waypoint> waypoints){
        // Defining waypoint Arraylist
        this.waypoints = waypoints;

        // Define Splinegenerater, All Splines Necessary
        SplineGenerator splinegen = new SplineGenerator();

        // Find Total Time to Make Sure Nothing is Going Wrong on Pose Calls
        totaltime = waypoints.get(waypoints.size()-1).time;

        for (int i = 0; i < waypoints.size()-1; i++) {
            // Get Relevant waypoints
            Waypoint waypoint1 = waypoints.get(i);
            Waypoint waypoint2 = waypoints.get(i+1);

            // Define All Variables For Spline
            double startx = waypoint1.x;
            double starty = waypoint1.y;
            double endx = waypoint2.x;
            double endy = waypoint2.y;
            double starttheta = waypoint1.theta;
            double endtheta = waypoint2.theta;
            double startxdot = waypoint1.xdot;
            double endxdot = waypoint2.xdot;
            double startydot = waypoint1.ydot;
            double endydot = waypoint2.ydot;
            double startxdotdot = waypoint1.xdotdot;
            double endxdotdot = waypoint2.xdotdot;
            double startydotdot = waypoint1.ydotdot;
            double endydotdot = waypoint2.ydotdot;
            double time = waypoint2.time-waypoint1.time;

            // Make Sure waypoints Are Correct
            assert waypoint2.time>waypoint1.time: "Waypoint times are not correct";

            // Generate Splines and Add Them to Array
            Spline[] segment = splinegen.SplineBetweenTwoPoints(startx, starty, endx, endy, starttheta, endtheta,
                    startxdot, endxdot, startydot, endydot, startxdotdot, endxdotdot, startydotdot, endydotdot, time);
            splines.add(segment);

            // Add Time
            waypointTimes.add(waypoint2.time);
        }
    }

    public Pose getRobotPose(double time){
        int splineindex = 0;

        if(totaltime <= time){
            splineindex = waypointTimes.size() - 1;
        } else{
            for (int i = 0; i < waypointTimes.size(); i++) {
                if(waypointTimes.get(i) > time){
                    splineindex = i;
                    break;
                }
            }
        }
        double splinetime = time - waypoints.get(splineindex).time;
        Spline[] currentspline = splines.get(splineindex);
        double x = currentspline[0].position(splinetime);
        double y = currentspline[1].position(splinetime);
        double theta = Math.atan2(currentspline[1].velocity(splinetime), currentspline[0].velocity(splinetime));
        return new Pose(x, y, theta);
    }
}
