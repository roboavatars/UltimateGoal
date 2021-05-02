package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Shooter Test")
@Config
public class ShooterTest extends LinearOpMode {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    public static double p1 = 40;
    public static double f1 = 0;
    public static double p2 = 6.25;
    public static double f2 = 1.4;
    public static double pidThresh = 100;
    public static int velocity = 2100;
    public static boolean on = true;

    @Override
    public void runOpMode() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        Servo feed = hardwareMap.get(Servo.class, "feedServo");
        Servo mag = hardwareMap.get(Servo.class, "magServo");

        waitForStart();

        while (opModeIsActive()) {
            if (on) {
                shooter1.setVelocity(velocity);
                shooter2.setVelocity(-velocity);
            } else {
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
            }

            mag.setPosition(Constants.MAG_SHOOT_POS);

            if (gamepad1.left_bumper) {
                feed.setPosition(Constants.FEED_HOME_POS);
            } else {
                feed.setPosition(Constants.FEED_TOP_POS);
            }

            if (Math.abs(velocity - shooter1.getVelocity()) > pidThresh) {
                shooter1.setVelocityPIDFCoefficients(p1, 0, 0, f1);
                shooter2.setVelocityPIDFCoefficients(p1, 0, 0, f1);
            } else {
                shooter1.setVelocityPIDFCoefficients(p2, 0, 0, f2);
                shooter2.setVelocityPIDFCoefficients(p2, 0, 0, f2);
            }

            addPacket("S1 Velo", shooter1.getVelocity());
            addPacket("S2 Velo", shooter2.getVelocity());
            addPacket("S1 Pos", shooter1.getCurrentPosition());
            addPacket("S2 Pos", shooter2.getCurrentPosition());
            addPacket("S1 Power", shooter1.getPower());
            addPacket("S2 Power", shooter2.getPower());
            addPacket("Coeffs", shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            addPacket("Target V", velocity);

            sendPacket();
        }
    }
}