package org.firstinspires.ftc.teamcode.Tests.FlickerTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.RobotClasses.Shooter;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp
@Config
//@Disabled
public class DoubleFlickerTest extends LinearOpMode {
    public static double bottomPos = Constants.FEED_MID_POS;
    public static double homePos = Constants.FEED_HOME_POS;
    public static double topPos = Constants.FEED_TOP_POS;
    public static int pos = 1;
    public static boolean magUp = false;
    public static boolean debug = false;
    private double position;

    private boolean magToggle = false;
    private boolean shootToggle = false;
    private long shootTime;
    private int delay = 4;
    public static int period = 250;

    private double x = 87;
    private double y = 63;
    public static double thetaLeft = 1.627;
    public static double thetaMid = 1.571;
    public static double thetaRight = 1.504;

    private Shooter shooter;
    private Intake intake;
    private Robot robot;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "feedServo");
        shooter = new Shooter(this);
        intake = new Intake(this, false);
        robot = new Robot(this, 87, 63, PI/2, false);

        intake.sticksHalf();
        intake.sticksUpdate();

        waitForStart();

        while(opModeIsActive()) {
            if (debug) {
                if (pos == 0) {
                    position = bottomPos;
                } else if (pos == 1) {
                    position = homePos;
                } else if (pos == 2) {
                    position = topPos;
                }
                servo.setPosition(position);

                if (magUp) {
                    shooter.magShoot();
                } else {
                    shooter.magHome();
                }
            } else {
                // Intake on/off/rev
                if (gamepad1.right_trigger > 0) {
                    intake.on();
                } else if (gamepad1.left_trigger > 0) {
                    intake.reverse();
                } else {
                    intake.off();
                }

                // Toggle flywheel/mag for shoot/home position
                if (gamepad1.x && !shootToggle) {
                    shootToggle = true;
                    if (shooter.magHome) {
                        shooter.magShoot();
                        shooter.flywheelPS();
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
                        if (robot.isAtPose(x, y, thetaLeft)) {
                            position = topPos;
                            delay++;
                        } else {
                            robot.setTargetPoint(x, y, thetaLeft);
                        }
                    } else if (delay == 1) {
                        if (robot.isAtPose(x, y, thetaMid)) {
                            position = bottomPos;
                            delay++;
                        } else {
                            robot.setTargetPoint(x, y, thetaMid);
                        }
                    } else if (delay == 2) {
                        if (robot.isAtPose(x, y, thetaRight)) {
                            position = topPos;
                            delay++;
                        } else {
                            robot.setTargetPoint(x, y, thetaRight);
                        }
                    } else if (delay == 3) {
                        position = homePos;
                        delay++;
                    }
                    servo.setPosition(position);
                }
            }

            addPacket("delay", delay);
            robot.update();
        }
    }
}