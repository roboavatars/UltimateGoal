package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Shooter Test")
@Config
public class ShooterTest extends LinearOpMode {

    private DcMotorEx shooter;

    public static double p = 80;
    public static double d = 0;
    public static double f = 13;
    public static int velocity = 1700;
    public static boolean on = true;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "flywheel");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

//        Servo mag = hardwareMap.get(Servo.class, "magServo");

        waitForStart();

        while (opModeIsActive()) {
            shooter.setVelocityPIDFCoefficients(p, 0, d, f);

            if (on) {
                shooter.setVelocity(velocity);
            } else {
                shooter.setPower(0);
            }

//            mag.setPosition(Constants.MAG_SHOOT_POS);

            addPacket("Velocity", shooter.getVelocity());
            addPacket("Position", shooter.getCurrentPosition());
            addPacket("Coeffs", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            addPacket("Target V", velocity);

            sendPacket();
        }
    }
}