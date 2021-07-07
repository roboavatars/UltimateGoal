package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Turret Test")
@Config
public class TurretTest extends LinearOpMode {

    private DcMotorEx turret;

    public static double RADIANS_PER_TICK = 1;
    public static double targetTheta = PI/2;

    public static double p = 40;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static boolean targetMode = true;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));

            if (targetMode) {
                turret.setTargetPosition((int) (targetTheta / RADIANS_PER_TICK));
            } else {
                turret.setPower(gamepad1.left_stick_x);
            }

            addPacket("Theta", turret.getTargetPosition() * RADIANS_PER_TICK);
            addPacket("Ticks", turret.getCurrentPosition());
            addPacket("Coeffs", turret.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            sendPacket();
        }
    }
}