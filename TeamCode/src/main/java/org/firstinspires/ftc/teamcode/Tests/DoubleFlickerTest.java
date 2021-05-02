package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

@TeleOp
@Config
//@Disabled
public class DoubleFlickerTest extends LinearOpMode {
    public static double homePos = Constants.FEED_HOME_POS;
    public static double topPos = Constants.FEED_TOP_POS;
    public static int pos = 1;
    public static boolean magUp = true;
    public static boolean debug = false;
    private double position;

    private boolean shootToggle = false;
    private boolean flywheelOn = false;
    private int delay = 4;

    public static int velocity = 1680;

    public static double thetaLeft = 90;
    public static double thetaMid = 1.81;
    public static double thetaRight = 1.73;

    private Robot robot;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "feedServo");
        robot = new Robot(this, 111, 63, PI/2, false);

        waitForStart();
        robot.shooter.magShoot();

        while(opModeIsActive()) {
            if (debug) {
                if (pos == 1) {
                    position = homePos;
                } else if (pos == 2) {
                    position = topPos;
                }
                servo.setPosition(position);

                if (magUp) {
                    robot.shooter.magShoot();
                } else {
                    robot.shooter.magHome();
                }
            } else {
                if (gamepad1.a) {
                    robot.intake.blockerUp();
                } else {
                    robot.intake.blockerDown();
                }

                if (gamepad1.x && !shootToggle) {
                    shootToggle = true;
                    if (flywheelOn) {
                        robot.shooter.setVelocity(velocity);
                    } else {
                        robot.shooter.flywheelOff();
                    }
                    flywheelOn = !flywheelOn;
                } else if (!gamepad1.x && shootToggle) {
                    shootToggle = false;
                }

                if (gamepad1.b) {
                    robot.shooter.feedHome();
                } else {
                    robot.shooter.feedTop();
                }

                if (gamepad1.left_trigger > 0) {
                    robot.intake.on();
                } else if (gamepad1.right_trigger > 0) {
                    robot.intake.reverse();
                } else {
                    robot.intake.off();
                }

                if (gamepad1.dpad_left) {
                    robot.setTargetPoint(new Target(111, 63, thetaLeft).thetaKp(3.0).thetaKd(0.10).xyKp(0.45).xyKd(0.04));
                } else if (gamepad1.dpad_up) {
                    robot.setTargetPoint(new Target(111, 63, thetaMid).thetaKp(3.0).thetaKd(0.10).xyKp(0.45).xyKp(0.04));
                } else if (gamepad1.dpad_right) {
                    robot.setTargetPoint(new Target(111, 63, thetaRight).thetaKp(3.0).thetaKd(0.10).xyKp(0.45).xyKp(0.04));
                } else {
                    robot.drivetrain.setControls(-gamepad1.left_stick_y , -gamepad1.left_stick_x , -gamepad1.right_stick_x);
                }
            }

            addPacket("delay", delay);
            robot.update();
        }
    }
}