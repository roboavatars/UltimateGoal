package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Intake
    public static double L_HOME_POS = 1;
    public static double L_OUT_POS = 0.23;
    public static double R_HOME_POS = 0.25;
    public static double R_OUT_POS = 0.9;

    // Shooter
    public static int HIGH_GOAL_VELOCITY = 1150;
    public static int POWERSHOT_VELOCITY = 875;

    public static double FLAP_HOME_POS = 0;
    public static double FLAP_MAX_POS = 0.15;

    public static double MAG_HOME_POS = 0.27;
    public static double MAG_VIBRATE_POS = 0.26;
    public static double MAG_SHOOT_POS = 0.495;

    public static double FEED_HOME_POS = 0.15;
    public static double FEED_MID = 0.2;
    public static double FEED_SHOOT_POS = 0.37;

    // Wobble Arm
    public static int WOBBLE_UP_POS = -80;
    public static int WOBBLE_DOWN_POS = -830;
    public static int WOBBLE_UP_TELEOP_POS = -80;
    public static int WOBBLE_DOWN_TELEOP_POS = -1000;

    public static double CLAMP_WOBBLE_POS = 0.20;
    public static double UNCLAMP_WOBBLE_POS = 0.75;
}