package org.firstinspires.ftc.teamcode.Tests.FlickerTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Shooter;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp
@Config
@Disabled
public class TopFlickerTest extends LinearOpMode {

    public static double midPos = Constants.FEED_MID_POS;
    public static double homePos = Constants.FEED_HOME_POS;
    public static double topPos = Constants.FEED_TOP_POS;
    public static int pos = 1;
    public static boolean debug = false;
    private double position;

    private boolean magToggle = false;
    private boolean shootToggle = false;
    private long shootTime;
    private int delay = 6;
    public static int period = 200;

    private Shooter shooter;
    private Intake intake;

    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.get(Servo.class, "feedServo");
        shooter = new Shooter(this);
        intake = new Intake(this, false);

        waitForStart();

        while(opModeIsActive()) {
            if (debug) {
                if (pos == 0) {
                    position = midPos;
                } else if (pos == 1) {
                    position = homePos;
                } else if (pos == 2) {
                    position = topPos;
                }
                servo.setPosition(position);
            } else {

                // Intake on/off/rev
                if (gamepad1.right_trigger > 0) {
                    intake.on();
                } else if (gamepad1.left_trigger > 0) {
                    intake.reverse();
                } else {
                    intake.off();
                }

                // Toggle mag for shoot/home position
                if (gamepad1.y && !magToggle) {
                    magToggle = true;
                    if (shooter.magHome) {
                        shooter.magShoot();
                    } else {
                        shooter.magHome();
                    }
                } else if (!gamepad1.x && magToggle) {
                    magToggle = false;
                }

                // Toggle flywheel/mag for shoot/home position
                if (gamepad1.x && !shootToggle) {
                    shootToggle = true;
                    if (shooter.magHome) {
                        shooter.magShoot();
                        shooter.flywheelHG();
                    } else {
                        shooter.magHome();
                        shooter.flywheelOff();
                    }
                } else if (!gamepad1.x && shootToggle) {
                    shootToggle = false;
                }

                // Reset flicker
                if (gamepad1.a) {
                    if (delay == 6) {
                        delay = 0;
                    }
                }

                // Flicker flicks after period milliseconds
                if (delay == 0 || (delay != 6 && System.currentTimeMillis() - shootTime > period)) {
                    shootTime = System.currentTimeMillis();
                    if (delay == 0) {
                        position = midPos;
                        delay++;
                    } else if (delay == 1) {
                        position = homePos;
                        delay++;
                    } else if (delay == 2) {
                        position = midPos;
                        delay++;
                    } else if (delay == 3) {
                        position = homePos;
                        delay++;
                    } else if (delay == 4) {
                        position = midPos;
                        delay++;
                    } else if (delay == 5) {
                        position = homePos;
                        delay++;
                    }
                    servo.setPosition(position);
                }
            }
            addPacket("delay", delay);
            sendPacket();
        }
    }
}