package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Intake
    public static double L_HOME_POS = 1;
    public static double L_HALF_POS = 0.77;
    public static double L_QUARTER_POS = 0.36;
    public static double L_OUT_POS = 0.23;

    public static double R_HOME_POS = 0.25;
    public static double R_HALF_POS = 0.60;
    public static double R_QUARTER_POS = 0.75;
    public static double R_OUT_POS = 0.9;

    public static double BLOCKER_UP_POS = 0.4;
    public static double BLOCKER_DOWN_POS = 1;

    // Shooter
    public static int HIGH_GOAL_VELOCITY = 1800;
    public static int POWERSHOT_VELOCITY = 1600;

    public static double FLAP_HOME_POS = 0.5;

    public static double MAG_HOME_POS = 0.28;
    public static double MAG_SHOOT_POS = 0.50;

    public static double FEED_HOME_POS = 0.9;
    public static double FEED_MID_POS = 0.9;
    public static double FEED_TOP_POS = 0;

    // Wobble Arm
    public static int WOBBLE_UP_POS = -50;
    public static int WOBBLE_DOWN_POS = -520;

    public static double CLAMP_WOBBLE_POS = 0.20;
    public static double UNCLAMP_WOBBLE_POS = 0.75;
}