package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Intake
    public static double BLOCKER_HOME_POS = 0.4;
    public static double BLOCKER_VERTICAL_POS = 0.68;

    public static double BUMPER_OUT_POS = 0.1;
    public static double BUMPER_HOME_POS = 1;

    public static double INTAKE_POWER = 1;
    public static double TRANSFER_POWER = 0.6;

    // Shooter
    public static int HIGH_GOAL_VELOCITY = 1620;
    public static int POWERSHOT_VELOCITY = 1430;

    public static double MAG_HOME_POS = 0.43;
    public static double MAG_SHOOT_POS = 0.85;

    public static double FEED_HOME_POS = 0.86;
    public static double FEED_SHOOT_POS = 0.73;

    // Wobble
    public static double WOBBLE_INSIDE_POS = 0.95;
    public static double WOBBLE_UP_POS = 0.625;
    public static double WOBBLE_DOWN_POS = 0.26;

    public static double WOBBLE_CLAMP_POS_LEFT = 0.95;
    public static double WOBBLE_UNCLAMP_POS_LEFT = 0.3;
    public static double WOBBLE_HOME_POS_LEFT = 0;

    public static double WOBBLE_CLAMP_POS_RIGHT = 0.1;
    public static double WOBBLE_UNCLAMP_POS_RIGHT = 0.6;
    public static double WOBBLE_HOME_POS_RIGHT = 1;
}