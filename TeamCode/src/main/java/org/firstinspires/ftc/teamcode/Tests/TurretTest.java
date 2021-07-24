package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Turret Test")
@Config
public class TurretTest extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private DcMotorEx turret;
    private double targetTheta = 0;

    public static final double TICKS_PER_RADIAN = 466.2 / PI;
    public static double a_NumFactor = 0;
    public static double b_DemonFactor = 2;
    public static double initialTheta = 0;

    public static double p = 2.25;
    public static double d = 5.5;
    public static double f = 0;

    public static boolean dashTarget = true;

    private double turretTheta;
    private double turretError;
    private double turretErrorChange;
    private double prevTime;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drivetrain = new MecanumDrivetrain(this, 87, 63, PI/2);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.updatePose();

            targetTheta = ((a_NumFactor * PI / b_DemonFactor) - drivetrain.theta + PI/2) % (2*PI);
            if (targetTheta < 0) {
                targetTheta += 2*PI;
            }
            if (targetTheta > 3*PI/2) {
                targetTheta -= 2*PI;
            }
            targetTheta = Math.min(Math.max(targetTheta, -PI/4), 7*PI/6);
            turretTheta = turret.getCurrentPosition() / TICKS_PER_RADIAN + initialTheta;
            turretErrorChange = targetTheta - turretTheta - turretError;
            turretError = targetTheta - turretTheta;

            if (dashTarget) {
                turret.setPower(p * turretError + d * turretErrorChange + f);
            } else {
                turret.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            drivetrain.setControls(-gamepad1.left_stick_y * 0.7, -gamepad1.left_stick_x * 0.7, -gamepad1.right_stick_x * 0.7);

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            drawRobot(drivetrain.x, drivetrain.y, drivetrain.theta, drivetrain.theta + turretTheta - PI/2, "black", "gray");
            addPacket("Target Theta", targetTheta);
            addPacket("Turret Theta", turretTheta);
            addPacket("Theta Error", turretError);
            addPacket("Turret Global", drivetrain.theta + turretTheta - PI/2);
            addPacket("IMU Theta", drivetrain.theta);
            addPacket("Ticks", turret.getCurrentPosition());
            addPacket("Power", turret.getPower());
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            sendPacket();

            telemetry.addData("Target Theta", targetTheta);
            telemetry.addData("Turret Theta", turretTheta);
            telemetry.addData("Theta Error", turretError);
            telemetry.addData("IMU", drivetrain.theta);
            telemetry.addData("Ticks", turret.getCurrentPosition());
            telemetry.addData("Power", turret.getPower());
            telemetry.update();
        }
    }
}