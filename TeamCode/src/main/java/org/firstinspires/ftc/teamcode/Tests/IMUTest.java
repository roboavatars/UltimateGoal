package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.IMU;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "IMU Test")
public class IMUTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrivetrain dt = new MecanumDrivetrain(this, 111, 63, PI/2);
        IMU imu = new IMU(PI/2, this);

        double prevTime = 0;

        waitForStart();

        while (opModeIsActive()) {

            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            imu.updateHeading();

            if (gamepad1.x) {
                imu.resetHeading(PI/2);
            }

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            addPacket("theta", imu.getTheta());
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            drawRobot(111, 63, imu.getTheta());
            drawField();
            sendPacket();
        }
    }
}
