package org.firstinspires.ftc.teamcode.Tests.T265Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.T265;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "T265 Auto")
@Disabled
public class T265Auto extends LinearOpMode {

    private double startX = 111;
    private double startY = 57;
    private double startTheta = Math.PI/2;
    private double x;
    private double y;
    private double theta;

    @Override
    public void runOpMode() {
//        MecanumDrivetrain dt = new MecanumDrivetrain(this, startX, startY, startTheta);
        T265 t265 = new T265(this, startX, startY, startTheta);

        waitForStart();
        t265.startCam();

        double counter = 0;

        while(opModeIsActive()) {
            counter++;

            if (counter >= 170000) {
                t265.stopCam();
            }
            if (counter >= 220000) {
                break;
            }

//            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            t265.updateCamPose();
            x = t265.getCamX();
            y = t265.getCamY();
            theta = t265.getCamTheta();

            drawRobot(x, y, theta, "red");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("counter", counter);
            addPacket("isEmpty", t265.isEmpty);
            sendPacket();

            telemetry.addData("X: " , x);
            telemetry.addData("Y: " , y);
            telemetry.addData("Theta: " , theta);
            telemetry.update();
        }

        t265.stopCam();
    }
}