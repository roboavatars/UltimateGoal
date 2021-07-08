package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "0 Turret Test")
@Config
public class TurretTest extends LinearOpMode {

    private DcMotorEx turret;
    private double targetTheta = 0;

    public static double TICKS_PER_RADIAN = 126 / PI;
    public static double a_NumFactor = 0;
    public static double b_DemonFactor = 1;

//    public static double power = 0.25;

    public static double p = 0.125;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static boolean dashTarget = true;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition((int) (targetTheta * TICKS_PER_RADIAN));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {

            targetTheta = a_NumFactor * PI / b_DemonFactor;
            double turretTheta = turret.getCurrentPosition() / TICKS_PER_RADIAN;
            double turretError = targetTheta - turretTheta;

            if (dashTarget) {
                // run to pos only supports p
//                turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f));

//                targetTheta += 0.5 * gamepad1.left_stick_x;
//                power = Math.max(-0.5, Math.min(power + 0.1 * gamepad1.right_stick_y, 0.5));

                turret.setTargetPosition((int) (targetTheta * TICKS_PER_RADIAN));
                turret.setPower(Math.max(-0.5, Math.min(p * turretError, 0.5)));
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                turret.setPower(0.4 * gamepad1.left_stick_x);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            addPacket("Target Theta", targetTheta);
            addPacket("Turret Theta", turretTheta);
            addPacket("Theta Error", turretError);
            addPacket("Ticks", turret.getCurrentPosition());
            addPacket("Power", turret.getPower());
//            addPacket("Coeffs", turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            sendPacket();

            telemetry.addData("Target Theta", targetTheta);
            telemetry.addData("Turret Theta", turretTheta);
            telemetry.addData("Ticks", turret.getCurrentPosition());
            telemetry.addData("Power", turret.getPower());
            telemetry.update();
        }
    }
}