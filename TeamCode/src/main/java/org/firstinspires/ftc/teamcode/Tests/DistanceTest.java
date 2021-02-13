package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp @Config
public class DistanceTest extends LinearOpMode {

    private DistanceSensor ringSensor;
    public static double zeroDist = 3.4;
    public static double oneDist = 2.8;
    public static double twoDist = 2;
    public static double threeDist = 1.4;

    @Override
    public void runOpMode() {
        ringSensor = hardwareMap.get(DistanceSensor.class, "ringSensor");

        waitForStart();

        while (opModeIsActive()) {

            double distance = getDistance();
            Log.w("distance", distance+"");

            addPacket("Distance", distance);
            addPacket("# rings", getNumRings());
            sendPacket();
        }
    }

    public double getDistance() {
        return ringSensor.getDistance(DistanceUnit.INCH);
    }

    public int getNumRings() {
        double dist = getDistance();
        if (dist > zeroDist) return 0;
        else if (dist > oneDist) return 1;
        else if (dist > twoDist) return 2;
        else return 3;
    }
}