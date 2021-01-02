package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Shooter;

@TeleOp
@Config
public class DoubleFlickerTest extends LinearOpMode {

    public static double topShoot = 0.95;
    public static double home = 0.05;
    public static double bottomShoot = 0.25;
    public static int pos = 1;
    public static boolean debug = true;
    private double position;

    private boolean shootToggle = false;
    private long shootTime;
    private int delay = 4;
    public static int period = 200;

    private Shooter shooter;

    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.get(Servo.class, "feedServo");
        shooter = new Shooter(this);

        waitForStart();

        while(opModeIsActive()) {
            if (debug) {
                if (pos == 0) {
                    position = topShoot;
                } else if (pos == 2) {
                    position = bottomShoot;
                } else if (pos == 1) {
                    position = home;
                }
                servo.setPosition(position);
            } else {

                // Toggle flywheel/mag for shoot/home position
                if (gamepad1.x && !shootToggle) {
                    shootToggle = true;
                    if (shooter.magHome) {
                        shooter.magShoot();
                        shooter.flywheelHighGoal();
                    } else {
                        shooter.magHome();
                        shooter.flywheelOff();
                    }
                } else if (!gamepad1.x && shootToggle) {
                    shootToggle = false;
                }

                // Reset flicker
                if (gamepad1.a) {
                    if (delay == 4) {
                        delay = 0;
                    }
                }

                // Flicker flicks after period milliseconds
                if (delay == 0 || (delay != 4 && System.currentTimeMillis() - shootTime > period)) {
                    shootTime = System.currentTimeMillis();
                    if (delay == 0) {
                        position = topShoot;
                        delay++;
                    } else if (delay == 1) {
                        position = bottomShoot;
                        delay++;
                    } else if (delay == 2) {
                        position = topShoot;
                        delay++;
                    } else if (delay == 3) {
                        position = home;
                        delay++;
                    }
                    servo.setPosition(position);
                }
            }
        }
    }
}