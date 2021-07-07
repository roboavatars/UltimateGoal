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

    public static double TICKS_PER_RADIAN = 126 / PI;
    public static double targetTheta = 0;
    public static double power = 0.1;

    public static double p = 40;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition((int) (targetTheta * TICKS_PER_RADIAN));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
//            turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f));

            targetTheta += 0.5 * gamepad1.left_stick_x;
            power = Math.max(-0.5, Math.min(power + 0.1 * gamepad1.right_stick_y, 0.5));
            turret.setTargetPosition((int) (targetTheta * TICKS_PER_RADIAN));
            turret.setPower(power);

            addPacket("Target Theta", targetTheta);
            addPacket("Turret Theta", turret.getCurrentPosition() / TICKS_PER_RADIAN);
            addPacket("Theta Input", 0.1 * gamepad1.left_stick_x);
            addPacket("Ticks", turret.getCurrentPosition());
            addPacket("Power", turret.getPower());
            addPacket("Power Input", 0.1 * gamepad1.right_stick_y);
//            addPacket("Coeffs", turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            sendPacket();

            telemetry.addData("Target Theta", targetTheta);
            telemetry.addData("Turret Theta", turret.getCurrentPosition() / TICKS_PER_RADIAN);
            telemetry.addData("Theta Input", 0.5 * gamepad1.left_stick_x);
            telemetry.addData("Ticks", turret.getCurrentPosition());
            telemetry.addData("Power", turret.getPower());
            telemetry.addData("Power Input", 0.1 * gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}