package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.T265;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "T265 Test")
public class T265Test extends LinearOpMode {

    private T265 t265;
    private double x, y, theta;

    @Override
    public void runOpMode() {
        t265 = new T265(this, 57, 135, 0);

        waitForStart();
        t265.startCam();

        while(opModeIsActive()){
            t265.updateCamPose();
            x = t265.getCamX();
            y = t265.getCamY();
            theta = t265.getCamTheta();

            drawRobot(x, y, theta, "black");
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