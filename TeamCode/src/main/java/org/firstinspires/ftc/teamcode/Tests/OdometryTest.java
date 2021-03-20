package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Odometry Test")
public class OdometryTest extends LinearOpMode {

    private double x, y, theta, prevTime;

    @Override
    public void runOpMode() {
        MecanumDrivetrain dt = new MecanumDrivetrain(this, 90, 9, Math.PI/2);

        waitForStart();

        while(opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            if (gamepad1.x) {
                dt.resetOdo(90, 9, Math.PI/2);
            }

            dt.updatePose();
            x = dt.x;
            y = dt.y;
            theta = dt.theta;

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            drawRobot(x, y, theta, "green");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            addPacket("diff", timeDiff);
            addPacket("pod1", dt.pod1);
            addPacket("pod2", dt.pod2);
            addPacket("pod3", dt.pod3);
            sendPacket();

            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addData("Theta: ", theta);
            telemetry.update();
        }
    }
}