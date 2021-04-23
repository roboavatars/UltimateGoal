package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Intake
    public static double L_HOME_POS = 0.15;
    public static double L_HALF_POS = 0.4;
    public static double L_SHOOT_POS = 0.6;
    public static double L_OUT_POS = 1;

    public static double R_HOME_POS = 0.5;
    public static double R_HALF_POS = 0.2;
    public static double R_SHOOT_POS = 0.1;
    public static double R_OUT_POS = 0;

    public static double BLOCKER_HOME_POS = 0.72;
    public static double BLOCKER_UP_POS = 0.49;
    public static double BLOCKER_DOWN_POS = 0.25;

    // Shooter
    public static int HIGH_GOAL_VELOCITY = 2050;
    public static int POWERSHOT_VELOCITY = 1530;

    public static double MAG_HOME_POS = 0.15;
    public static double MAG_SHOOT_POS = 0.36;

    public static double FEED_HOME_POS = 0.86;
    public static double FEED_TOP_POS = 0.14;

    public static double FLAP_DOWN_POS = 0.45;
    public static double FLAP_UP_POS = 0.70;

    public static double ZERO_DIST = 4.4;
    public static double ONE_DIST = 3.9;
    public static double TWO_DIST = 3.3;
    public static double THREE_DIST = 2.5;

    // Wobble
    public static double WOBBLE_UP_POS = 0.73;
    public static double WOBBLE_DOWN_POS = 0.19;

    public static double WOBBLE_CLAMP_POS = 0.9;
    public static double WOBBLE_UNCLAMP_POS = 0.3;
}