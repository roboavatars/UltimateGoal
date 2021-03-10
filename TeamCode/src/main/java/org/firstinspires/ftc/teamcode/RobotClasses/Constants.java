package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Intake
    public static double L_HOME_POS = 1;
    public static double L_HALF_POS = 0.77;
    public static double L_QUARTER_POS = 0.39;
    public static double L_OUT_POS = 0.23;

    public static double R_HOME_POS = 0.25;
    public static double R_HALF_POS = 0.58;
    public static double R_QUARTER_POS = 0.75;
    public static double R_OUT_POS = 0.85;

    public static double BLOCKER_UP_POS = 0.51;
    public static double BLOCKER_DOWN_POS = 0.2;

    // Shooter
    public static int HIGH_GOAL_VELOCITY = 1850;
    public static int POWERSHOT_VELOCITY = 1560;

    public static double FLAP_HOME_POS = 0.5;

    public static double MAG_HOME_POS = 0.29;
    public static double MAG_SHOOT_POS = 0.50;

    public static double FEED_HOME_POS = 0;
    public static double FEED_MID_POS = 0.2;
    public static double FEED_TOP_POS = 0.9;

    // Wobble Arm
    public static int WOBBLE_UP_POS = -20;
    public static int WOBBLE_DOWN_POS = -630;

    public static double CLAMP_WOBBLE_POS = 0.84;
    public static double UNCLAMP_WOBBLE_POS = 0.3;
}