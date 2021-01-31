package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Shooter;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp
@Config
//@Disabled
public class DoubleFlickerTest extends LinearOpMode {

    public static double bottomPos = Constants.FEED_MID_POS;
    public static double homePos = Constants.FEED_HOME_POS;
    public static double topPos = Constants.FEED_TOP_POS;
    public static int pos = 1;
    public static boolean debug = false;
    private double position;

    private boolean magToggle = false;
    private boolean shootToggle = false;
    private long shootTime;
    private int delay = 4;
    public static int period = 200;

    private Shooter shooter;
    private Intake intake;

    @Override
    public void runOpMode() {

        CRServo servo = hardwareMap.get(CRServo.class, "feedServo");
        shooter = new Shooter(this);
        intake = new Intake(this, false);

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
                servo.setPower(1);
            } else {

                // Intake on/off/rev
                if (gamepad1.right_trigger > 0) {
                    intake.intakeOn();
                } else if (gamepad1.left_trigger > 0) {
                    intake.intakeRev();
                } else {
                    intake.intakeOff();
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
                    if (delay == 4) {
                        delay = 0;
                    }
                }

                // Flicker flicks after period milliseconds
                if (delay == 0 || (delay != 4 && System.currentTimeMillis() - shootTime > period)) {
                    shootTime = System.currentTimeMillis();
                    servo.setPower(1);
                    delay++;
                }
                servo.setPower(0);
            }
            addPacket("delay", delay);
            sendPacket();
        }
    }
}