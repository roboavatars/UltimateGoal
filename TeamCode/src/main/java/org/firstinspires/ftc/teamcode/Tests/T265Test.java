package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.T265;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "T265 Test")
public class T265Test extends LinearOpMode {

    private double startX = 111, startY = 57, startTh = Math.PI/2;
    private double x, y, theta;

    @Override
    public void runOpMode() {
        MecanumDrivetrain dt = new MecanumDrivetrain(this, startX, startY, startTh);
        T265 t265 = new T265(this, startX, startY, startTh);

        waitForStart();
        t265.startCam();

        while(opModeIsActive()){

            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            t265.updateCamPose();
            x = t265.getCamX();
            y = t265.getCamY();
            theta = t265.getCamTheta();

            drawRobot(x, y, theta, "red");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            sendPacket();

            telemetry.addData("X: " , x);
            telemetry.addData("Y: " , y);
            telemetry.addData("Theta: " , theta);
            telemetry.update();
        }

        t265.stopCam();
    }
}