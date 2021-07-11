package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotClasses.IMU;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "0 Turret Test")
@Config
public class TurretTest extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private DcMotorEx turret;
    private IMU imu;
    private double targetTheta = 0;

    public static double TICKS_PER_RADIAN = 126 / PI;
    public static double a_NumFactor = 0;
    public static double b_DemonFactor = 2;

    public static double p = 1;
    public static double i = 0;
    public static double d = 4.8;
    public static double f = 0;

    public static boolean dashTarget = true;

    private double turretTheta;
    private double turretError;
    private double turretErrorChange;
    private double prevTime;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = new IMU(PI/2, this);
        drivetrain = new MecanumDrivetrain(this, 102, 54, PI/2);

        waitForStart();

        while (opModeIsActive()) {
            imu.updateHeadingUncapped();

            targetTheta = PI - imu.getTheta() + a_NumFactor * PI / b_DemonFactor;
            turretTheta = turret.getCurrentPosition() / TICKS_PER_RADIAN;
            turretErrorChange = targetTheta - turretTheta - turretError;
            turretError = targetTheta - turretTheta;

            if (dashTarget) {
                turret.setPower(Math.max(-0.5, Math.min(p * turretError + d * turretErrorChange + f, 0.5)));
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                turret.setPower(0.4 * gamepad1.left_stick_x);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            drivetrain.setControls(-gamepad1.left_stick_y * 0.7, -gamepad1.left_stick_x * 0.7, -gamepad1.right_stick_x * 0.7);

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            drawRobot(102, 54, imu.getTheta(), imu.getTheta() + turretTheta, "black", "gray");
            addPacket("Target Theta", targetTheta);
            addPacket("Turret Theta", turretTheta);
            addPacket("Theta Error", turretError);
            addPacket("IMU", imu.getTheta());
            addPacket("Ticks", turret.getCurrentPosition());
            addPacket("Power", turret.getPower());
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            sendPacket();

            telemetry.addData("Target Theta", targetTheta);
            telemetry.addData("Turret Theta", turretTheta);
            telemetry.addData("Theta Error", turretError);
            telemetry.addData("IMU", imu.getTheta());
            telemetry.addData("Ticks", turret.getCurrentPosition());
            telemetry.addData("Power", turret.getPower());
            telemetry.update();
        }
    }
}