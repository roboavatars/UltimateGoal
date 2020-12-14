package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp @Config
public class ShooterTest extends LinearOpMode {

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    public static double p = 28;
    public static double i = 0;
    public static double d = 0;
    public static double f = 18;
    public static int velocity = -875;
    public static boolean on = false;

    @Override
    public void runOpMode() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (on) {
                shooter1.setVelocity(velocity);
                shooter2.setVelocity(-velocity);
            } else {
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
            }

            shooter1.setVelocityPIDFCoefficients(p, i, d, f);
            shooter2.setVelocityPIDFCoefficients(p, i, d, f);

            addPacket("Shooter 1", shooter1.getVelocity());
            addPacket("Shooter 2", shooter2.getVelocity());
            sendPacket();
        }
    }
}