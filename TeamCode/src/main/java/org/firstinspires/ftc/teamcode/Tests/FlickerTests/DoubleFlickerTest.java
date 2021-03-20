package org.firstinspires.ftc.teamcode.Tests.FlickerTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp
@Config
@Disabled
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

    public static double thetaLeft = 1.627;
    public static double thetaMid = 1.571;
    public static double thetaRight = 1.504;

    private Robot robot;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "feedServo");
        robot = new Robot(this, 87, 63, PI/2, false);

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
                    robot.shooter.magShoot();
                } else {
                    robot.shooter.magHome();
                }
            } else {
                if (gamepad2.a) {
                    robot.intake.blockerDown();
                } else {
                    robot.intake.blockerUp();
                }

                if (gamepad1.left_trigger > 0) {
                    robot.intake.on();
                } else if (gamepad1.right_trigger > 0) {
                    robot.intake.reverse();
                } else {
                    robot.intake.off();
                }
            }

            robot.drivetrain.setControls(-gamepad1.left_stick_y , -gamepad1.left_stick_x , -gamepad1.right_stick_x);

            addPacket("delay", delay);
            robot.update();
        }
    }
}