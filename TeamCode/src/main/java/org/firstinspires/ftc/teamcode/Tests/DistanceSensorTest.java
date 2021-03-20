package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Distance Sensor Test")
@Config
@Disabled
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor ringSensor;
    private Robot robot;
    private int numRings = 3;

    public static double zeroDist = 4.65;
    public static double oneDist = 4.0;
    public static double twoDist = 3.3;
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

            addPacket("Distance", getDistance());
            addPacket("# Rings", getNumRings());
            addPacket("Zero", zeroDist);
            addPacket("One", oneDist);
            addPacket("Two", twoDist);
            addPacket("Three", threeDist);
            sendPacket();

            telemetry.addData("Distance", robot.shooter.getDistance());
            telemetry.addData("# Rings", robot.shooter.getNumRings());
            telemetry.update();
        }
    }

    public double getDistance() {
        return ringSensor.getDistance(DistanceUnit.INCH);
    }

    public int getNumRings() {
        double dist = getDistance();
        int tempRings;
        if (dist > zeroDist) {
            tempRings = 0;
        } else if (dist > oneDist) {
            tempRings = 1;
        } else if (dist > twoDist) {
            tempRings = 2;
        } else {
            tempRings = 3;
        }

        if (tempRings != 3 || numRings >= 2) {
            numRings = tempRings;
        }
        return numRings;
    }
}