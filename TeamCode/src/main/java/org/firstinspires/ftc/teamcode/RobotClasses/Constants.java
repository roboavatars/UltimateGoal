package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Intake
    public static double L_HOME_POS = 1;
    public static double L_SWEEP_POS = 0.8;
    public static double L_COLLECT_POS = 0.5;
    public static double L_OUT_POS = 0;

    public static double R_HOME_POS = 0;
    public static double R_SWEEP_POS = 0.2;
    public static double R_COLLECT_POS = 0.5;
    public static double R_OUT_POS = 1;

    public static double BLOCKER_HOME_POS = 0.7;
    public static double BLOCKER_UP_POS = 0.5;
    public static double BLOCKER_DOWN_POS = 0.23;

    // Shooter
    public static int HIGH_GOAL_VELOCITY = 1850;
    public static int POWERSHOT_VELOCITY = 1550;

    public static double MAG_HOME_POS = 0.08;
    public static double MAG_SHOOT_POS = 0.27;

    public static double FEED_HOME_POS = 0;
    public static double FEED_MID_POS = 0.2;
    public static double FEED_TOP_POS = 0.9;

    public static double ZERO_DIST = 4.4;
    public static double ONE_DIST = 3.9;
    public static double TWO_DIST = 3.3;
    public static double THREE_DIST = 2.5;

    // Wobble
    public static double WOBBLE_UP_POS = 0.8;
    public static double WOBBLE_DOWN_POS = 0.23;

    public static double WOBBLE_CLAMP_POS = 0.55;
    public static double WOBBLE_UNCLAMP_POS = 0;
}