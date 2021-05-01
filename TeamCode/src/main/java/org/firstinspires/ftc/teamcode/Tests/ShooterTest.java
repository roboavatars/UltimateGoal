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

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    public static double p = 50;//6.5;
    public static double i = 0;
    public static double d = 0;
    public static double f = 12;//0;
    public static double velocity = 1950;
    public static boolean on = true;

    @Override
    public void runOpMode() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (on) {
                shooter1.setVelocity(velocity);
                shooter2.setVelocity(-velocity);
//                shooter1.setPower(-velocity);
//                shooter2.setPower(velocity);

//                shooter1.setPower(shooter2.getPower());
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            shooter1.setVelocityPIDFCoefficients(p, i, d, f);
            shooter2.setVelocityPIDFCoefficients(p, i, d, f);

            addPacket("S1 Velo", shooter1.getVelocity());
            addPacket("S2 Velo", shooter2.getVelocity());
            addPacket("S1 Pos", shooter1.getCurrentPosition());
            addPacket("S2 Pos", shooter2.getCurrentPosition());
            addPacket("S1 Power", shooter1.getPower());
            addPacket("S2 Power", shooter2.getPower());
            addPacket("Target V", velocity);

            sendPacket();
        }
    }
}