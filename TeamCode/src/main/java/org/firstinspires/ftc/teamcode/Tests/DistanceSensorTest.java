package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Distance Sensor Test") @Config
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor ringSensor;
    private Robot robot;
    private double distance;

    public static double zeroDist = 4.55;
    public static double oneDist = 3.9;
    public static double twoDist = 3.2;
    public static double threeDist = 2.5;

    @Override
    public void runOpMode() {
        ringSensor = hardwareMap.get(DistanceSensor.class, "ringSensor");
        robot = new Robot(this, 90, 9, PI/2, false);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                robot.intake.on();
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            double tempDist = getDistance();
            if (tempDist != 3 || distance >= 2) {
                distance = tempDist;
            }
            Log.w("distance", distance + "");

            addPacket("Distance", distance);
            addPacket("# Rings", getNumRings());
            addPacket("Zero", zeroDist);
            addPacket("One", oneDist);
            addPacket("Two", twoDist);
            addPacket("Three", threeDist);
            sendPacket();

            telemetry.addData("Distance", distance);
            telemetry.addData("# Rings", getNumRings());
            telemetry.update();
        }
    }

    public double getDistance() {
        return ringSensor.getDistance(DistanceUnit.INCH);
    }

    public int getNumRings() {
        double dist = getDistance();
        if (dist > zeroDist) {
            return 0;
        } else if (dist > oneDist) {
            return 1;
        } else if (dist > twoDist) {
            return 2;
        } else {
            return 3;
        }
    }
}