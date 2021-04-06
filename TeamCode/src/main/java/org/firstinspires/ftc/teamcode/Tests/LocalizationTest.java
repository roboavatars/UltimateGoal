package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.T265;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Localization Test")
public class LocalizationTest extends LinearOpMode {
    private double dtX, dtY, dtTheta;
    private double camX, camY, camTheta;
    private double x, y, theta;
    private double prevTime;

    private double startX = 111;
    private double startY = 63;
    private double startTheta = PI/2;

    @Override
    public void runOpMode() {
        MecanumDrivetrain dt = new MecanumDrivetrain(this, startX, startY, startTheta);
        T265 t265 = new T265(this, startX, startY, startTheta);

        waitForStart();
        t265.startCam();

        while(opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            dt.updatePose();
            t265.updateCamPose();

            dtX = dt.x;
            dtY = dt.y;
            dtTheta = dt.theta;

            camX = t265.getX();
            camY = t265.getY();
            camTheta = t265.getTheta();

            if (t265.confidence <= 1) {
                x = dtX;
                y = dtY;
                theta = dtTheta;
            } else if (t265.confidence == 2) {
                x = (dtX + camX) / 2;
                y = (dtY + camY) / 2;
                theta = (dtTheta + camTheta) / 2;
            } else if (t265.confidence == 3) {
                x = camX;
                y = camY;
                theta = camTheta;
            }

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            drawField();
            drawRobot(dtX, dtY, dtTheta, "green");
            drawRobot(camX, camY, camTheta, "red");
            drawRobot(x, y, theta, "black");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            sendPacket();
        }
    }
}