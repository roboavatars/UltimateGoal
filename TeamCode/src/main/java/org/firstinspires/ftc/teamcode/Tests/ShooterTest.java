package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Shooter Test")
@Config
public class ShooterTest extends LinearOpMode {
    private DcMotorEx shooter;
    private Servo flicker;

    public static double p1 = 40;
    public static double f1 = 0;
    public static double p2 = 6.25;
    public static double f2 = 1.4;
    public static double pidThresh = 100;
    public static int velocity = 2100;
    public static boolean on = true;
    public static boolean flick = true;
    public static int flickDist = 80;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        flicker = hardwareMap.get(Servo.class, "flicker");
        flicker.setPosition(1);

//        Servo feed = hardwareMap.get(Servo.class, "feedServo");
//        Servo mag = hardwareMap.get(Servo.class, "magServo");

        waitForStart();

        while (opModeIsActive()) {
            if (on) {
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
            }

            if (flick) {
                flicker.setPosition(0.8);
                sleep(flickDist);
                flicker.setPosition(0.98);
                sleep(flickDist);
            }

            if (!flick) {
                flicker.setPosition(flickDist);
            }

//            mag.setPosition(Constants.MAG_SHOOT_POS);
//
//            if (gamepad1.left_bumper) {
//                feed.setPosition(Constants.FEED_HOME_POS);
//            } else {
//                feed.setPosition(Constants.FEED_TOP_POS);
//            }

            if (Math.abs(velocity - shooter.getVelocity()) > pidThresh) {
                shooter.setVelocityPIDFCoefficients(p1, 0, 0, f1);
            } else {
                shooter.setVelocityPIDFCoefficients(p2, 0, 0, f2);
            }

            addPacket("S1 Velo", shooter.getVelocity());
            addPacket("S1 Pos", shooter.getCurrentPosition());
            addPacket("Coeffs", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            addPacket("Target V", velocity);

            sendPacket();
        }
    }
}